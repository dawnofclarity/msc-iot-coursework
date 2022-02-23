# msc-iot-coursework
Internet Of Things Coursework

## Assignment 1: Fire Alarm

Features:
- Check temp. and light levels periodically against a trigger threshold
- When triggered blinks the red LED and logs ACTIVE to console

Additional Features:
- A calibration mode measures the first 10 readings to derive an average
current value for each sensor, which is used to derive relevant trigger
thresholds. The green LED is on during calibration as a visual cue.
- Calibration can also be triggered by pressing the white button. Note that
entering calibration mode will silence the alarm if it's currently active.
- To mitigate against stray readings sounding the alarm, a multi-trigger
policy is used. When both thresholds are exceeded, a number of further
checks are carried out at a faster poll rate. The alarm is only sounded
if these further checks still indicate an alarm condition exists.
- When quiescent, the device uses a long poll period to help preserve
battery life.
- When the alarm is triggered a rate limited network message is sent via
unicast to the node at GATEWAY_ADDR with a simple message containing
the location and current readings. This message will repeat not more
than once per defined limit period, regardless of if the alarm continues
to sound or is re-triggered. This serves to preserve battery and mitigate
against network swamping.

## Assignment 2: Aggregation Algorithms

Features:
- Storage / Ring buffer
- Activity Measurement using Standard Deviation
- Aggregation - 12->2, 12->3, 12->4, 12->6, or as is
- Reporting - display buffer, SD, aggregation level, & output
- 
Additional Features:
- Derive the min/max of the data for each period
- Demonstrate how a compressed yet recoverable data set can be created.
- Derive the normalised Z values for the data
- Derive the PAA
- Convert to SAX
- Derive the distance between the current and the previous set of sax data.

## Acknowledgements:
- Contiki 2.6 API & Examples: http://contiki.sourceforge.net/docs/2.6/index.html
- Symbolic Aggregate approXimation (SAX): https://github.com/jMotif/SAX
