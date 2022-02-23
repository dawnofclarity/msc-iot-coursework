/*
 * Assignment 2: Aggregation Algorithms, Name: Dawn Raison, URN: 6609229
 * 
 * Features:
 * - Storage / Ring buffer
 * - Activity Measurement using Standard Deviation
 * - Aggregation - 12->2, 12->3, 12->4, 12->6, or as is
 * - Reporting - display buffer, SD, aggregation level, & output
 * 
 * Additional Features:
 * - Derive the min/max of the data for each period
 * - Demonstrate how a compressed yet recoverable data set can be created.
 * - Derive the normalised Z values for the data
 * - Derive the PAA
 * - Convert to SAX
 * - Derive the distance between the current and the previous set of sax data.
 * 
 * Acknowledgements:
 * - Contiki 2.6 API & Examples:
 *   http://contiki.sourceforge.net/docs/2.6/index.html
 * - Symbolic Aggregate approXimation (SAX)
 *   https://github.com/jMotif/SAX
 * 
 */
#include "contiki.h"
#include "lib/random.h"
#include "net/rime.h"

#include <stdio.h>
#include <math.h>

#include "dev/light-sensor.h"

/* Should match dDecimal() multiplier */
#define PRINTF_FLOAT_FMT "%d.%02u"

#define POLL_INTERVAL (CLOCK_SECOND / 4)

/*
 *  Data buffer size
 * - if changing this also update deriveAggregationFactor accordingly
 */
#define DATA_BUF_SIZE (12)

#define PAA_WIDTH (6)
#define PAA_COALESCE (DATA_BUF_SIZE / PAA_WIDTH)

#define SAX_BASE_SYMBOL 'a'

/* Alphabet size to use */
#define SAX_ALPHABET_SIZE (7)
#define SAX_MIN_ALPHABET (3)

typedef struct repeat_term_st {
    unsigned int repeat;
    unsigned int value;
} REPEAT_TERM_ST;


/* 
 * Data for ranges 3 to 10 symbols
 * Note: normally only the required set would need to be included, however
 * these are left here to allow other alphabet sizes to be demonstrated.
 * 
 * - derived from values in sax_to_20.xls at:
 *   https://cs.gmu.edu/~jessica/sax.htm
 */
static float sc3[] = {-0.43, 0.43};
static float sc4[] = {-0.67, 0, 0.67};
static float sc5[] = {-0.84, -0.25, 0.25, 0.84};
static float sc6[] = {-0.97, -0.43, 0, 0.43, 0.97};
static float sc7[] = {-1.07, -0.57, -0.18, 0.18, 0.57, 1.07};
static float sc8[] = {-1.15, -0.67, -0.32, 0, 0.32, 0.67, 1.15};
static float sc9[] = {-1.22, -0.76, -0.43, -0.14, 0.14, 0.43, 0.76, 1.22};
static float sc10[] = {-1.28, -0.84, -0.52, -0.25, 0, 0.25, 0.52, 0.84, 1.28};

static float * saxCutline[] = {
    sc3, sc4, sc5, sc6, sc7, sc8, sc9, sc10
};

static float saxDistance[SAX_ALPHABET_SIZE][SAX_ALPHABET_SIZE];
static float saxCompressionRatio = ((float) DATA_BUF_SIZE / PAA_WIDTH);

/* Events */
static process_event_t report_data_event;

/* Define processes */
PROCESS(mainProcess, "Main Process");
PROCESS(reportProcess, "Report Process");

AUTOSTART_PROCESSES(&mainProcess, &reportProcess);

/**
 * Derive the appropriate aggregation factor for the magnitude
 * of the standard deviation given.
 * Note: valid aggregation factors must divide equally into DATA_BUF_SIZE
 * 
 * @param stdDev
 * @return 
 */
unsigned int deriveAggregationFactor(float stdDev) {
    if (stdDev < 1.41) {
        return 6;
    }
    if (stdDev < 8) {
        return 4;
    }
    if (stdDev < 35) {
        return 3;
    }
    if (stdDev < 100) {
        return 2;
    }
    return 1;
}

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
    return (unsigned int) ((value - dWhole(value)) * 100);
}

/**
 * Quick implementation of Newton's algorithm to find a square root
 * 
 * - Babylonian_method / Newtons algorithm:
 *   https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method
 * - An implementation of Newton's algorithm in GO
 *   https://stackoverflow.com/questions/3581528/how-is-the-square-root-function-implemented
 * 
 * @param value
 * @return approximate sqrt of value
 */
float sqrtf(float value) {

    float z = 1.0F;
    int ii;

    for (ii = 1; ii <= 10; ii++) {
        z -= (z * z - value) / (2 * z);
    }
    return z;
}

/**
 * Quick square implementation
 * 
 * @param value
 * @return value squared
 */
float squaref(float value) {
    return value * value;
}

/**
 * print the buffer contents
 * 
 * @param measureBuffer
 */
void printBuffer(char * descriptor, float *buffer, int length) {
    int ii;

    printf("%s(", descriptor);
    for (ii = 0; ii < length; ii++) {
        float temp = buffer[ii];
        if (ii > 0) {
            printf(",");
        }
        printf(PRINTF_FLOAT_FMT, dWhole(temp), dDecimal(temp));
    }
    printf(")\n");
}

/**
 * Read light sensor and apply transfer function
 * 
 * @return light sensor reading
 */
float readSensor() {
    unsigned int lv = light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
    float vSensor = 1.5 * (float) lv / 4096.0;
    float I = vSensor / 100000;
    return 0.625 * 1e6 * I * 1000;
}

/**
 * Aggregate the source data based on the aggregation factor
 * Assumption: DATA_BUF_SIZE shall be evenly divisible by the aggregation
 * factor. If not, then only completely contained elements are considered.
 * 
 * @param factor        # of elements to be combined (1,2,3, etc.)
 * @param aggrBuffer    preallocated buffer for receiving the aggregated stream.
 * @param measureBuffer input measurements
 * @return 
 */
int aggregateData(int factor, float *aggrBuffer, float *measureBuffer) {
    float aggSum;
    int srcIx = 0;
    int aggIx = 0;
    int destIx = 0;

    /* Special "do nothing" case just mirrors in to out */
    if (factor < 2) {
        memcpy(aggrBuffer, measureBuffer, DATA_BUF_SIZE * sizeof (float));
        return DATA_BUF_SIZE;
    }

    /* just in case we're called with a non-evenly divisible value */
    unsigned int maxElements = (DATA_BUF_SIZE / factor) * factor;
    if (maxElements != DATA_BUF_SIZE) {
        printf("invalid aggregation value; maxElements restricted to: %u\n",
                maxElements);
    }

    /* aggregate by taking the mean of each aggregationFactor block */
    while (srcIx < maxElements) {
        aggIx = 0;
        aggSum = 0;
        while (aggIx < factor) {
            aggSum += measureBuffer[srcIx + aggIx];
            aggIx++;
        }
        aggrBuffer[destIx] = aggSum / aggIx;
        destIx++;
        srcIx += aggIx;
    }
    return destIx;
}

/**
 * Encode the buffer as a series of repeat terms
 * Note: For the purposes of demonstration, the sensor values are considered
 * here as unsigned integers; any fraction part is discarded.
 * 
 * @param repeatBuffer
 * @param measureBuffer
 * @return 
 */
int encodeAsRepeatTerms(REPEAT_TERM_ST *repeatBuffer, float *measureBuffer) {
    unsigned int ii = 0;
    unsigned int jj = 0;
    unsigned int kk = 0;
    unsigned int value;
    unsigned int nextValue;

    while (ii < DATA_BUF_SIZE) {
        jj = 1;
        value = (unsigned int) measureBuffer[ii];
        do {
            nextValue = (unsigned int) measureBuffer[ii + jj];
            if (value != nextValue) {
                break;
            }
            jj++;
        } while (ii + jj < DATA_BUF_SIZE);
        repeatBuffer[kk].repeat = jj;
        repeatBuffer[kk].value = value;
        kk++;
        ii += jj;
    }
    return kk;
}

/**
 * print repeat buffer
 * 
 * @param repeatBuffer buffer containing a list of repeat terms
 * @param number of terms in buffer
 */
void printRepeatBuffer(REPEAT_TERM_ST *buffer, int length) {
    unsigned int ii;

    printf("R(");
    for (ii = 0; ii < length; ii++) {
        if (ii > 0) {
            printf(",");
        }
        if (buffer[ii].repeat == 1) {
            printf("%u", buffer[ii].value);
        } else {
            printf("%ux%u", buffer[ii].repeat, buffer[ii].value);
        }
    }
    printf(")\n");
}

/**
 * encode the source buffer as a set of PAA terms
 * 
 * @param paaBuffer Output
 * @param sourceBuffer Input
 */
void encodeAsPaa(float *paaBuffer, float *sourceBuffer) {
    unsigned int ii = 0;
    unsigned int jj = 0;
    unsigned int kk = 0;
    float value;

    while (ii < DATA_BUF_SIZE) {
        value = 0;
        for (jj = 0; jj < PAA_COALESCE; jj++) {
            value += sourceBuffer[ii + jj];
        }
        paaBuffer[kk] = value / PAA_COALESCE;
        kk++;
        ii += PAA_COALESCE;
    }
}

/**
 * Convert an index into a sax symbol
 * 
 * @param ix
 * @return 
 */
char saxIxToSymbol(unsigned int ix) {
    return SAX_BASE_SYMBOL + ix;
}

/**
 * Convert a sax symbol into an index
 * 
 * @param symbol
 * @return 
 */
unsigned int saxSymbolToIx(char symbol) {
    return symbol - SAX_BASE_SYMBOL;
}

/**
 * Get the appropriate cutline for the current alphabet size
 * 
 * @return array of cutline thresholds 
 */
float * getCutline() {
    return saxCutline[SAX_ALPHABET_SIZE - SAX_MIN_ALPHABET];
}

/**
 * Convert a set of PAA data into a SAX representation of the data
 * 
 * @param saxBuffer
 * @param paaBuffer
 */
void paaToSax(char *saxBuffer, float *paaBuffer) {
    unsigned int ii;
    unsigned int jj;
    float value;
    float * cutline = getCutline();

    for (ii = 0; ii < PAA_WIDTH; ii++) {
        jj = 0;
        value = paaBuffer[ii];
        while (jj < (SAX_ALPHABET_SIZE - 1) && value > cutline[jj]) {
            jj++;
        }
        saxBuffer[ii] = saxIxToSymbol(jj);
    }
    saxBuffer[PAA_WIDTH] = 0;
}

/**
 * Initialise the SAX distance lookup table
 * This contains the relative distances between
 * any two given symbols.
 * These values are pre-squared for use in the distance function.
 */
void initSaxDistanceLookup() {
    unsigned int ii;
    unsigned int jj;
    float value;

    float * cutline = getCutline();

    printf("\nCutline[%u] ->", SAX_ALPHABET_SIZE);
    for (ii = 0; ii < SAX_ALPHABET_SIZE - 1; ii++) {
        printf(" " PRINTF_FLOAT_FMT,
                dWhole(cutline[ii]), dDecimal(cutline[ii]));
    }
    printf("\n");

    memset(saxDistance, 0, sizeof (saxDistance));

    /* Set up the SAX distance lookup table */
    for (ii = 0; ii < SAX_ALPHABET_SIZE; ii++) {
        for (jj = ii + 2; jj < SAX_ALPHABET_SIZE; jj++) {
            value = squaref(cutline[ii] - cutline[jj - 1]);
            saxDistance[ii][jj] = value;
            saxDistance[jj][ii] = value;
        }
    }

    printf("\nSax Distance Lookup table (squared!)\n   ");
    for (jj = 0; jj < SAX_ALPHABET_SIZE; jj++) {
        printf(" %c   ", saxIxToSymbol(jj));
    }
    printf("\n");

    for (ii = 0; ii < SAX_ALPHABET_SIZE; ii++) {
        printf("%c:", saxIxToSymbol(ii));
        for (jj = 0; jj < SAX_ALPHABET_SIZE; jj++) {
            value = saxDistance[ii][jj];
            printf(" " PRINTF_FLOAT_FMT,
                    dWhole(value), dDecimal(value));
        }
        printf("\n");
    }

}

/**
 * Derive the distance between two SAX words
 * 
 * @param sax1 SAX word 1
 * @param sax2 SAX word 2
 */
float deriveSaxDistance(char *sax1, char *sax2) {
    float value;
    unsigned int ii;

    value = 0;
    for (ii = 0; ii < PAA_WIDTH; ii++) {
        value += saxDistance[saxSymbolToIx(sax1[ii])][saxSymbolToIx(sax2[ii])];
    }
    /* NB sqrtf(a) * sqrtf(b) === sqrtf(a * b) */
    return sqrtf(saxCompressionRatio * value);
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
    static float measureBuffer[DATA_BUF_SIZE];
    static unsigned int dataBufferIx = 0;

    PROCESS_BEGIN();

    initSaxDistanceLookup();

    /* Allocate the required events */
    report_data_event = process_alloc_event();

    /* Activate the sensors */
    SENSORS_ACTIVATE(light_sensor);

    dataBufferIx = 0;

    /* Arm timer */
    etimer_set(&et, POLL_INTERVAL);

    while (1) {

        /* Wait for an event */
        PROCESS_WAIT_EVENT();

        /* If the event is a timer expiry */
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {

            /* Poll sensor and store in buffer */
            measureBuffer[dataBufferIx] = readSensor();
            dataBufferIx += 1;

            if (dataBufferIx == DATA_BUF_SIZE) {
                /* NB: If poll times are likely to faster that report
                 * processing time, we will need to send a COPY of the data
                 * here rather than a reference to prevent overwrites.*/
                process_post(&reportProcess, report_data_event, measureBuffer);

                dataBufferIx = 0;
            }

            /* Re-arm timer */
            etimer_reset(&et);
        }
    }

    PROCESS_END();
}

/**
 * Report process
 * Called when the buffer is full, it then crunches the numbers,
 * and prints the desired output to the console.
 * 
 * @param process_pt    Process Identifier
 * @param ev            Event identifier
 * @param data          Opaque payload for events
 * @return n/a
 */
PROCESS_THREAD(reportProcess, ev, data) {

    static char lastSaxBuffer[PAA_WIDTH + 1];

    float *measureBuffer;
    float aggrBuffer[DATA_BUF_SIZE];
    unsigned int aggrFactor;
    int numAggregated;
    REPEAT_TERM_ST repeatBuffer[DATA_BUF_SIZE];
    int numRepeats;
    float normalizedBuffer[DATA_BUF_SIZE];
    float paaBuffer[PAA_WIDTH];
    char saxBuffer[PAA_WIDTH + 1];
    float minMeasure;
    float maxMeasure;
    int ii;
    float sum;
    float mean;
    float sumSquares;
    float stdDev;
    float saxDistance;

    PROCESS_BEGIN();

    /* set a default for first iteration */
    memset(lastSaxBuffer,
            saxIxToSymbol(SAX_ALPHABET_SIZE / 2), sizeof (lastSaxBuffer));

    /* Loop forever */
    while (1) {

        /* Wait for an event */
        PROCESS_WAIT_EVENT_UNTIL(ev == report_data_event);

        /* Recover data ptr */
        measureBuffer = (float *) data;

        // --------------------------------------------------
        // Calculate stuff
        // --------------------------------------------------

        /* compute mean */
        sum = 0;
        for (ii = 0; ii < DATA_BUF_SIZE; ii++) {
            sum += measureBuffer[ii];
        }
        mean = sum / DATA_BUF_SIZE;

        /* compute stdDev */
        sumSquares = 0;
        for (ii = 0; ii < DATA_BUF_SIZE; ii++) {
            sumSquares += squaref(measureBuffer[ii] - mean);
        }
        stdDev = sqrtf(sumSquares / DATA_BUF_SIZE);

        /* aggregate */
        aggrFactor = deriveAggregationFactor(stdDev);
        numAggregated = aggregateData(aggrFactor, aggrBuffer, measureBuffer);

        /* create a set of normalised values */
        for (ii = 0; ii < DATA_BUF_SIZE; ii++) {
            normalizedBuffer[ii] = (measureBuffer[ii] - mean) / stdDev;
        }

        /* encode as PAA */
        encodeAsPaa(paaBuffer, normalizedBuffer);

        /* PAA to SAX */
        paaToSax(saxBuffer, paaBuffer);

        /* distance from last */
        saxDistance = deriveSaxDistance(lastSaxBuffer, saxBuffer);

        /* encode as repeats */
        numRepeats = encodeAsRepeatTerms(repeatBuffer, measureBuffer);

        /* compute min/max */
        minMeasure = measureBuffer[0];
        maxMeasure = measureBuffer[0];
        for (ii = 1; ii < DATA_BUF_SIZE; ii++) {
            float value = measureBuffer[ii];
            if (value < minMeasure) {
                minMeasure = value;
            }
            if (value > maxMeasure) {
                maxMeasure = value;
            }
        }

        // --------------------------------------------------
        // Print stuff
        // --------------------------------------------------
        printf("\n");
        printBuffer("B", measureBuffer, DATA_BUF_SIZE);

        printf("StdDev: " PRINTF_FLOAT_FMT "\n",
                dWhole(stdDev), dDecimal(stdDev));
        printf("Aggregation factor: %u\n", aggrFactor);
        printBuffer("X", aggrBuffer, numAggregated);

        printf("Min: " PRINTF_FLOAT_FMT ", Max: " PRINTF_FLOAT_FMT "\n",
                dWhole(minMeasure), dDecimal(minMeasure),
                dWhole(maxMeasure), dDecimal(maxMeasure));

        printBuffer("Z", normalizedBuffer, DATA_BUF_SIZE);
        printBuffer("P", paaBuffer, PAA_WIDTH);
        printf("S: %s  dist(%s): " PRINTF_FLOAT_FMT "\n",
                saxBuffer, lastSaxBuffer,
                dWhole(saxDistance), dDecimal(saxDistance));

        printRepeatBuffer(repeatBuffer, numRepeats);

        /* save for next loop */
        memcpy(lastSaxBuffer, saxBuffer, sizeof (lastSaxBuffer));
    }

    PROCESS_END();
}

