#include <HomeSeerClient.h>

const short temperatureProbePin = A0;
const short flowMeterPin = 2;
const short bucketWaterLevelPin = 3;
const short reservoirInletValvePin = 5;
const short alarmNoFlowPin = 6;
const short bucketReservoirValvePin = 7;
const short bottomSensorPin = 22;
const short topSensorPin = 23;

float calibrationFactorFlow = 4.5;
byte sensorInterrupt = 0;
volatile byte pulseCount;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

unsigned long previousFlowSensorMillis;

unsigned long previousTemperatureSensorMillis;
const long temperatureSensorInterval = 10*60*1000L;

unsigned long previousBucketLevelMillis;
const long bucketLevelInterval = 5*1000L;

unsigned long previousAliveMillis;
const long aliveInterval = 3*60*1000L;

unsigned long previousReservoirLevelMillis;
const long reservoirLevelInterval = 5*1000L;

HomeSeerClient homeSeerClient;

// HomeSeer id's
const int reservoirValveId = 1425;
const int reservoirWaterInletId = 1426;
const int flowRateId = 1428;
const int totalVolumeId = 1429;
const int reservoirTemperatureId = 1432;
const int reservoirControllerId = 1427;
const int bucketReservoirId = 2056;

#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000
int samples[NUMSAMPLES];

bool fillValveOpen = false;
bool reservoirStatusUpdatedInHS = false;
int topValue = 0;
int bottomValue = 0;

void setup()
{
    pinMode(topSensorPin, INPUT);
    pinMode(bottomSensorPin, INPUT);
    
    pinMode(flowMeterPin, INPUT);
    digitalWrite(flowMeterPin, HIGH);
    
    pinMode(alarmNoFlowPin, OUTPUT);
    digitalWrite(alarmNoFlowPin, LOW);
    
    pinMode(reservoirInletValvePin, OUTPUT);
    digitalWrite(reservoirInletValvePin, HIGH);

    pinMode(bucketReservoirValvePin, OUTPUT);
    digitalWrite(bucketReservoirValvePin, HIGH);

    pinMode(bucketWaterLevelPin, INPUT);
    
    pulseCount = 0;
    flowRate = 0.0;
    flowMilliLitres = 0;
    totalMilliLitres = 0;
    previousReservoirLevelMillis = 0;
    previousFlowSensorMillis = 0;
    previousTemperatureSensorMillis = 0;
    previousAliveMillis = 0;
    previousBucketLevelMillis = 0;

    analogReference(EXTERNAL);
    Serial.begin(9600);
    while (!Serial) continue;

    homeSeerClient.init();
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    delay(10000);
}

void loop() {
    checkReservoirLevel();
    if(fillValveOpen)
        checkWaterFlow();
    checkBucketLevel();
    checkTemperature();
    sendAlive();
}

void checkBucketLevel() {
    if(millis() - previousBucketLevelMillis >= bucketLevelInterval) {
        bool bucketEmpty = digitalRead(bucketWaterLevelPin);

        if(bucketEmpty == true)
        {
            digitalWrite(bucketReservoirValvePin, LOW);
            homeSeerClient.updateHomeSeer(bucketReservoirId, 100);
        }
        else
        {
            digitalWrite(bucketReservoirValvePin, HIGH);
            homeSeerClient.updateHomeSeer(bucketReservoirId, 0);
        }
        previousBucketLevelMillis += bucketLevelInterval;
    }
}

void checkReservoirLevel() {
    if(millis() - previousReservoirLevelMillis >= reservoirLevelInterval) {
        topValue = digitalRead(topSensorPin);
        delay(200);
        bottomValue = digitalRead(bottomSensorPin);

        if (topValue == LOW && bottomValue == HIGH)
        {
            if(fillValveOpen)
            {
                // Close valve
                digitalWrite(reservoirInletValvePin, HIGH);
                homeSeerClient.updateHomeSeer(reservoirValveId, 0);
                homeSeerClient.updateHomeSeer(reservoirWaterInletId, 0);
                fillValveOpen = false;
                //Reset possible alarms on water flow
                digitalWrite(alarmNoFlowPin, LOW);
            }
        }

        if (topValue == HIGH && bottomValue == LOW)
        {
            if(!fillValveOpen)
            {
                // Open valve
                digitalWrite(reservoirInletValvePin, LOW);
                if(!reservoirStatusUpdatedInHS) {
                    homeSeerClient.updateHomeSeer(reservoirValveId, 100);
                    homeSeerClient.updateHomeSeer(reservoirWaterInletId, 100);
                }
                reservoirStatusUpdatedInHS = true;
                fillValveOpen = true;
            }
        }
        previousReservoirLevelMillis += reservoirLevelInterval;
    }
}

void sendAlive() {
    if(millis() - previousAliveMillis >= aliveInterval) {
        previousAliveMillis += aliveInterval;
        homeSeerClient.updateHomeSeer(reservoirControllerId, 100);
    }
}

void checkTemperature() {
    if(millis() - previousTemperatureSensorMillis >= temperatureSensorInterval) {
        uint8_t i;
        float average;
        
        // take N samples in a row, with a slight delay
        for (i=0; i< NUMSAMPLES; i++) {
            samples[i] = analogRead(THERMISTORPIN);
            delay(10);
        }
        
        // average all the samples out
        average = 0;
        for (i=0; i< NUMSAMPLES; i++) {
            average += samples[i];
        }
        average /= NUMSAMPLES;
        
        // convert the value to resistance
        average = 1023 / average - 1;
        average = SERIESRESISTOR / average;
        
        float steinhart;
        steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
        steinhart = log(steinhart);                  // ln(R/Ro)
        steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                 // Invert
        steinhart -= 273.15;                         // convert to C
        Serial.print("Temperature "); 
        Serial.print(steinhart);
        Serial.println(" *C");
        previousTemperatureSensorMillis += temperatureSensorInterval;
        homeSeerClient.updateHomeSeer(reservoirTemperatureId, steinhart);
        delay(1000);
    }
}

void checkWaterFlow() {
    if(millis() - previousFlowSensorMillis > 1000)    // Only process counters once per second
    { 
        // Disable the interrupt while calculating flow rate and sending the value to
        // the host
        detachInterrupt(sensorInterrupt);
            
        // Because this loop may not complete in exactly 1 second intervals we calculate
        // the number of milliseconds that have passed since the last execution and use
        // that to scale the output. We also apply the calibrationFactor to scale the output
        // based on the number of pulses per second per units of measure (litres/minute in
        // this case) coming from the sensor.
        flowRate = ((1000.0 / (millis() - previousFlowSensorMillis)) * pulseCount) / calibrationFactorFlow;
        if((int)flowRate == 0)
        {
            digitalWrite(alarmNoFlowPin, HIGH);
            homeSeerClient.updateHomeSeer(reservoirWaterInletId, 100);
        }
        else
        {
            digitalWrite(alarmNoFlowPin, LOW);
            homeSeerClient.updateHomeSeer(reservoirWaterInletId, 0);
        }
        
        // Note the time this processing pass was executed. Note that because we've
        // disabled interrupts the millis() function won't actually be incrementing right
        // at this point, but it will still return the value it was set to just before
        // interrupts went away.
        previousFlowSensorMillis = millis();
        
        // Divide the flow rate in litres/minute by 60 to determine how many litres have
        // passed through the sensor in this 1 second interval, then multiply by 1000 to
        // convert to millilitres.
        flowMilliLitres = (flowRate / 60) * 1000;
        
        // Add the millilitres passed in this second to the cumulative total
        totalMilliLitres += flowMilliLitres;
        
        // Print the flow rate for this second in litres / minute
        // Serial.print("Flow rate: ");
        // Serial.print(int(flowRate));  // Print the integer part of the variable
        // Serial.print(" L/min");
        // Serial.print("\t"); 		  // Print tab space
        homeSeerClient.updateHomeSeer(flowRateId, (int)flowRate);
        // Print the cumulative total of litres flowed since starting
        // Serial.print("Output Liquid Quantity: ");        
        // Serial.print(totalMilliLitres);
        // Serial.println(" mL"); 
        // Serial.print("\t"); 		  // Print tab space
        // Serial.print(totalMilliLitres/1000);
        // Serial.print(" L");
        homeSeerClient.updateHomeSeer(totalVolumeId, (int)totalMilliLitres/1000);

        // Reset the pulse counter so we can start incrementing again
        pulseCount = 0;
        
        // Enable the interrupt again now that we've finished sending output
        attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }
}

void pulseCounter() {
  // Increment the pulse counter
  pulseCount++;
}