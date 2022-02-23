/*
 * Assignment 1: Fire Alarm, Name: Dawn Raison, URN: 6609229
 * 
 * Features:
 * - Check temp. and light levels periodically against a trigger threshold
 * - When triggered blinks the red LED and logs ACTIVE to console
 * 
 * Additional Features:
 * - A calibration mode measures the first 10 readings to derive an average
 *   current value for each sensor, which is used to derive relevant trigger
 *   thresholds. The green LED is on during calibration as a visual cue.
 * - Calibration can also be triggered by pressing the white button. Note that
 *   entering calibration mode will silence the alarm if it's currently active.
 * - To mitigate against stray readings sounding the alarm, a multi-trigger
 *   policy is used. When both thresholds are exceeded, a number of further
 *   checks are carried out at a faster poll rate. The alarm is only sounded
 *   if these further checks still indicate an alarm condition exists.
 * - When quiescent, the device uses a long poll period to help preserve
 *   battery life.
 * - When the alarm is triggered a rate limited network message is sent via
 *   unicast to the node at GATEWAY_ADDR with a simple message containing
 *   the location and current readings. This message will repeat not more
 *   than once per defined limit period, regardless of if the alarm continues
 *   to sound or is re-triggered. This serves to preserve battery and mitigate
 *   against network swamping.
 * 
 * Acknowledgements:
 * - Contiki 2.6 API & Examples:
 *   http://contiki.sourceforge.net/docs/2.6/index.html
 * 
 */
#include "contiki.h"
#include "lib/random.h"
#include "net/rime.h"

#include <stdio.h>

#include "dev/light-sensor.h"
#include "dev/sht11-sensor.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"

/* Should match dDecimal() multiplier */
#define PRINTF_FLOAT_FMT "%d.%03u"

/* Temperature default, and threshold */
#define ALARM_TEMP (28.0)
#define ALARM_TEMP_OFFSET (2.0)

/* Light level default, and threshold */
#define ALARM_LIGHT (280.0)
#define ALARM_LIGHT_OFFSET (20.0)

/* Calibration step count */
#define CALIBRATE_STEPS 10

/* Pre-trigger count */
#define PRE_TRIGGER_STEPS 3

/* Intervals depending on current state */
#define CALIBRATION_INTERVAL (CLOCK_SECOND)
#define QUIESCENT_ACTIVE_INTERVAL (CLOCK_SECOND * 10)
#define PRE_TRIGGER_ACTIVE_INTERVAL (CLOCK_SECOND)
#define TRIGGERED_ACTIVE_INTERVAL (CLOCK_SECOND / 2)

/* Network configuration */
#define MIN_NETWORK_SEND_INTERVAL (CLOCK_SECOND * 60)
#define NETWORK_CHANNEL (26)
#define GATEWAY_ADDR_0 (220)
#define GATEWAY_ADDR_1 (149)

/* Events */
static process_event_t report_data_event;
static process_event_t network_send_event;

/* Somewhere to track our state */
typedef struct alarm_state_st {
    /* trigger temp for alarm */
    float tempTrigger;

    /* last measured value for temp */
    float tempMeasure;

    /* cumulative count used when calibrating the temp sensor */
    float tempCumulative;

    /* flag denoting the temp was exceeded at the last measure */
    int tempExceeded;

    /* trigger light level for alarm */
    float lightTrigger;

    /* last measured value for light level  */
    float lightMeasure;

    /* cumulative count used when calibrating the light level sensor */
    float lightCumulative;

    /* flag denoting the light level was exceeded at the last measure */
    int lightExceeded;

    /* down counter for number of calibration steps remaining
     * 0 denotes calibration complete */
    int calibrating;

    /* counter to track preTrigger events, in this way we only sound the
     * alarm after a number of confirmed checks to help prevent false alarms */
    int preTrigger;

    /* flag denoting alarm is currently active / triggered */
    int active;
} ALARM_STATE;

/* Define processes */
PROCESS(mainProcess, "Main Process");
PROCESS(reportProcess, "Reporting Process");
PROCESS(networkProcess, "Network Process");

AUTOSTART_PROCESSES(&reportProcess, &mainProcess, &networkProcess);

/**
 * Grab the Whole part of a float
 * @param value
 * @return 
 */
unsigned int dWhole(float value) {
    return (int) value;
}

/**
 * Grab the decimal part of a float
 * @param value
 * @return 
 */
unsigned int dDecimal(float value) {
    if (value < 0) {
        value = -value;
    }
    return (unsigned int) ((value - dWhole(value)) * 1000);
}

/**
 * Reset the given state to empty / calibrating.
 * @param pState
 */
void initState(ALARM_STATE *pState) {
    pState->calibrating = CALIBRATE_STEPS;
    pState->preTrigger = 0;
    pState->active = 0;
    pState->tempTrigger = ALARM_TEMP;
    pState->tempMeasure = 0;
    pState->tempExceeded = 0;
    pState->tempCumulative = 0;
    pState->lightTrigger = ALARM_LIGHT;
    pState->lightMeasure = 0;
    pState->lightCumulative = 0;
    pState->lightExceeded = 0;
}

/**
 * Poll the temperature sensor and apply transfer function
 * @param pState
 */
void pollTempSensor(ALARM_STATE *pState) {
    // Poll temperature and apply transfer function
    unsigned int tv = sht11_sensor.value(SHT11_SENSOR_TEMP);
    pState->tempMeasure = ((float) tv * 0.01) - 39.6;
}

/**
 * Poll the light sensor and apply transfer function
 * @param pState
 */
void pollLightSensor(ALARM_STATE *pState) {
    // Poll light sensor
    unsigned int lv = light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC);
    float vSensor = 1.5 * (float) lv / 4096.0;
    float I = vSensor / 100000;
    pState->lightMeasure = 0.625 * 1e6 * I * 1000;
}

/**
 * Derive the next timer period based on the current state of the alarm
 * @param pState
 * @return Timer period
 */
unsigned int deriveTimerPeriod(ALARM_STATE *pState) {
    if (pState->active) {
        return TRIGGERED_ACTIVE_INTERVAL;
    }
    if (pState->preTrigger > 0) {
        return PRE_TRIGGER_ACTIVE_INTERVAL;
    }
    if (pState->calibrating > 0) {
        return CALIBRATION_INTERVAL;
    }
    return QUIESCENT_ACTIVE_INTERVAL;
}

/**
 * Process the sensor data and decide what we should do next
 * @param pState
 */
void processSensorData(ALARM_STATE * pState) {
    if (pState->calibrating > 0) {
        /* Calibrating */
        pState->tempCumulative += pState->tempMeasure;
        pState->lightCumulative += pState->lightMeasure;

        pState->calibrating -= 1;

        if (pState->calibrating == 0) {
            /* Calibration complete, derive new trigger levels */
            pState->tempTrigger = (pState->tempCumulative / CALIBRATE_STEPS)
                    + ALARM_TEMP_OFFSET;
            pState->lightTrigger = (pState->lightCumulative / CALIBRATE_STEPS)
                    + ALARM_LIGHT_OFFSET;
        }
    } else {
        /* Active Monitoring */
        pState->tempExceeded = pState->tempMeasure >= pState->tempTrigger;
        pState->lightExceeded = pState->lightMeasure >= pState->lightTrigger;

        /* Are both sensors are triggered */
        if (pState->tempExceeded && pState->lightExceeded) {
            /* Count the required number of pre-trigger events */
            if (pState->preTrigger < PRE_TRIGGER_STEPS) {
                pState->preTrigger += 1;
            } else {
                /* Sound alarm */
                pState->active = 1;
            }
        } else {
            /* Reset */
            pState->preTrigger = 0;
            pState->active = 0;
        }
    }
}

/**
 * Main process and control loop
 * 
 * @param process_pt    Process Identifier
 * @param ev            Event identifier
 * @param data          Opaque payload for events
 * @return n/a
 */
PROCESS_THREAD(mainProcess, ev, data) {
    static struct etimer et;
    static ALARM_STATE alarmState;

    PROCESS_BEGIN();

    /* Allocate the required events */
    report_data_event = process_alloc_event();
    network_send_event = process_alloc_event();

    /* Initial conditions */
    initState(&alarmState);
    leds_off(LEDS_ALL);

    /* Activate the sensors */
    SENSORS_ACTIVATE(button_sensor);
    SENSORS_ACTIVATE(light_sensor);
    SENSORS_ACTIVATE(sht11_sensor);

    /* Arm timer */
    etimer_set(&et, deriveTimerPeriod(&alarmState));

    while (1) {

        /* Wait for an event */
        PROCESS_WAIT_EVENT();

        /* If the button was pressed */
        if (ev == sensors_event && data == &button_sensor) {
            /* restart the calibration process */
            initState(&alarmState);

            /* Re-arm timer - duration may have changed */
            etimer_set(&et, deriveTimerPeriod(&alarmState));
        }
        /* If the event is a timer expiry */
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {

            /* Poll sensors */
            pollTempSensor(&alarmState);
            pollLightSensor(&alarmState);

            processSensorData(&alarmState);

            /* Re-arm timer */
            etimer_set(&et, deriveTimerPeriod(&alarmState));
        }
        process_post(&reportProcess, report_data_event, &alarmState);
    }

    PROCESS_END();
}

/**
 * Report process
 * This listens for 'report_data_event's from the main process
 * then updates the LEDs and emits messages accordingly.
 * For clarity the 3 potential actions are treated separately.
 * 
 * @param process_pt    Process Identifier
 * @param ev            Event identifier
 * @param data          Opaque payload for events
 * @return n/a
 */
PROCESS_THREAD(reportProcess, ev, data) {

    ALARM_STATE *pState;

    PROCESS_BEGIN();

    while (1) {
        /* Wait for timer */
        PROCESS_WAIT_EVENT_UNTIL(ev == report_data_event);

        /* recover ptr to data */
        pState = (ALARM_STATE *) data;

        /* ------------------------------------------------------------*/
        /* LEDs                                                        */
        /* ------------------------------------------------------------*/

        /* Show the green LED while calibrating */
        if (pState->calibrating > 0) {
            leds_on(LEDS_GREEN);
        } else {
            leds_off(LEDS_GREEN);
        }

        /*
         * Show a blinking red light if alarm is active.
         * Note: We can't use led_toggle() as it can leave the invert
         * register in the wrong state rendering leds_off() useless
         */
        if (pState->active && (leds_get() & LEDS_RED) == 0) {
            leds_on(LEDS_RED);
        } else {
            leds_off(LEDS_RED);
        }

        /* ------------------------------------------------------------*/
        /* CONSOLE Messages                                            */
        /* ------------------------------------------------------------*/
        if (pState->calibrating > 0) {
            printf("CAL: ");
        }

        /* output temp and threshold values */
        printf("t: " PRINTF_FLOAT_FMT " / ",
                dWhole(pState->tempMeasure), dDecimal(pState->tempMeasure));
        printf("t: " PRINTF_FLOAT_FMT " ",
                dWhole(pState->tempTrigger), dDecimal(pState->tempTrigger));
        if (pState->tempExceeded) {
            printf("*");
        }

        /* output light and threshold values */
        printf("t: " PRINTF_FLOAT_FMT " / ",
                dWhole(pState->lightMeasure), dDecimal(pState->lightMeasure));
        printf("t: " PRINTF_FLOAT_FMT " ",
                dWhole(pState->lightTrigger), dDecimal(pState->lightTrigger));
        if (pState->lightExceeded) {
            printf("*");
        }

        /* output the alarm state */
        if (pState->active) {
            /* It's a bit hot around here */
            printf(", ACTIVE\n");
        } else {
            /* Normality has been restored... */
            printf(", OK\n");
        }

        /* ------------------------------------------------------------*/
        /* Network                                                     */
        /* ------------------------------------------------------------*/
        if (pState->active) {
            /* forward the request on to the network process */
            process_post(&networkProcess, network_send_event, pState);
        }
    }

    PROCESS_END();
}

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from) {
    printf("message received [%u.%u] '%s'\n",
            from->u8[0], from->u8[1], (char *) packetbuf_dataptr());
}

static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

/**
 * Network process
 * Listens for a network send event, sends a message, then sleeps for a period
 * to ensure we 
 * a) don't swamp the network with alarm messages
 * b) preserve battery (network sends are expensive)
 * The blue LED will blink once when a message is sent
 * 
 * @param process_pt    Process Identifier
 * @param ev            Event identifier
 * @param data          Opaque payload for events
 * @return 
 */
PROCESS_THREAD(networkProcess, ev, data) {

    static struct etimer et;
    rimeaddr_t addr;
    ALARM_STATE *pState;
    char buffer[40];

    PROCESS_BEGIN();

    while (1) {

        /* Wait for a request to send something */
        PROCESS_WAIT_EVENT_UNTIL(ev == network_send_event);

        /* recover ptr to data */
        pState = (ALARM_STATE *) data;

        /*
         * Create a simple formatted message
         * type, addr1, addr2, temp, light
         */
        size_t len = snprintf(buffer, sizeof (buffer),
                "alarm,%u,%u," PRINTF_FLOAT_FMT "," PRINTF_FLOAT_FMT,
                rimeaddr_node_addr.u8[0],
                rimeaddr_node_addr.u8[1],
                dWhole(pState->tempMeasure), dDecimal(pState->tempMeasure),
                dWhole(pState->lightMeasure), dDecimal(pState->lightMeasure)
                );
        packetbuf_copyfrom(buffer, len);

        leds_on(LEDS_BLUE);

        /* Send a message (unicast) */
        unicast_open(&uc, NETWORK_CHANNEL, &unicast_callbacks);

        addr.u8[0] = GATEWAY_ADDR_0;
        addr.u8[1] = GATEWAY_ADDR_1;
        unicast_send(&uc, &addr);

        unicast_close(&uc);

        leds_off(LEDS_BLUE);

        printf("Network Send complete (%s)\n", buffer);

        /* Now wait until the timer expires to control flow
         * (Other events are ignored) */
        etimer_set(&et, MIN_NETWORK_SEND_INTERVAL);
        PROCESS_WAIT_EVENT_UNTIL(
                ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
    }

    PROCESS_END();
}
