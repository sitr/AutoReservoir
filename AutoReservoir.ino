#include <Ethernet.h>
#include <SPI.h>
#include <ArduinoJson.h>

const short topSensorPin = 9;
const short bottomSensorPin = 8;
const short flowMeterPin = 2;
const short relayPin = 5;
const short alarmNoFlowPin = 6;
const short temperatureProbePin = A0;

float calibrationFactorFlow = 4.5;
byte sensorInterrupt = 0;
volatile byte pulseCount;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

unsigned long previousFlowSensorMillis;

unsigned long previousTemperatureSensorMillis;
const long temperatureSensorInterval = 60*1000L;

unsigned long previousAliveMillis;
const long aliveInterval = 3*60*1000L;

byte mac[] = { 0xDE, 0xAC, 0xEE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 86, 230);
IPAddress dnsServer(192, 168, 86, 1);
IPAddress homeSeerServer(192, 168, 86, 44);
EthernetClient client;

const unsigned long HTTP_TIMEOUT = 10000;

int reservoirValveId = 1425;
int reservoirWaterInletId = 1426;
int flowRateId = 1428;
int totalVolumeId = 1429;
int reservoirTemperatureId = 1432;
int reservoirControllerId = 1427;

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

void setup()
{
    pinMode(topSensorPin, INPUT);
    pinMode(bottomSensorPin, INPUT);
    pinMode(flowMeterPin, INPUT);
    digitalWrite(flowMeterPin, HIGH);
    pinMode(alarmNoFlowPin, OUTPUT);
    pinMode(relayPin, OUTPUT);

    pulseCount = 0;
    flowRate = 0.0;
    flowMilliLitres = 0;
    totalMilliLitres = 0;
    previousFlowSensorMillis = 0;
    previousTemperatureSensorMillis = 0;
    previousAliveMillis = 0;
    
    analogReference(EXTERNAL);
    Serial.begin(9600);
    while (!Serial) continue;
    setupEthernet();
    client.setTimeout(HTTP_TIMEOUT);
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
}

void loop() {
    checkWaterLevel();
    if(fillValveOpen)
        checkWaterFlow();
    checkTemperature();
    sendAlive();
}

void checkWaterLevel() {
    int topValue = digitalRead(topSensorPin);
    int bottomValue = digitalRead(bottomSensorPin);

    if (topValue == LOW && bottomValue == HIGH)
    {
        if(fillValveOpen)
        {
            // Close valve
            digitalWrite(relayPin, LOW);
            updateHomeSeer(reservoirValveId, 0);
            updateHomeSeer(reservoirWaterInletId, 0);
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
            digitalWrite(relayPin, HIGH);
            updateHomeSeer(reservoirValveId, 100);
            fillValveOpen = true;
        }
    }
}

void setupEthernet() {
    Serial.println("Initialize Ethernet with DHCP:");
    if (Ethernet.begin(mac) == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        if (Ethernet.hardwareStatus() == EthernetNoHardware){
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
            while (true) {
                delay(1);
            }
        }
        if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
        }
        Ethernet.begin(mac, ip, dnsServer);
    } 
    else {
        Serial.print("DHCP assigned IP ");
        Serial.println(Ethernet.localIP());
    }
}

void sendAlive() {
    if(millis() - previousAliveMillis >= aliveInterval) {
        previousAliveMillis += aliveInterval;
        updateHomeSeer(reservoirControllerId, 100);
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
        updateHomeSeer(reservoirTemperatureId, steinhart);
    }
}

bool connect() {
    Serial.print("Connect to ");
    Serial.println(homeSeerServer);

    bool ok = client.connect(homeSeerServer, 80);

    Serial.println(ok ? "Connected" : "Connection Failed!");
    return ok;
}

void disconnect() {
	Serial.println("Disconnect");
	client.stop();
}

void updateHomeSeer(int deviceId, double deviceValue) {
    if (connect()) {
        if (sendRequest(deviceId, deviceValue)) {
            if (readReponseContent()) {
                Serial.println("HomeSeer updated");
            }
        }
    }
    disconnect();
}

bool readReponseContent() {
      // Check HTTP status
    char status[32] = {0};
    client.readBytesUntil('\r', status, sizeof(status));
    if (strcmp(status, "HTTP/1.0 200 OK") != 0) {
        Serial.print(F("Unexpected response: "));
        Serial.println(status);
        return false;
    }
	return true;
}

// Skip HTTP headers so that we are at the beginning of the response's body
bool skipResponseHeaders() {
	// HTTP headers end with an empty line
	char endOfHeaders[] = "\r\n\r\n";
	bool ok = client.find(endOfHeaders);

	if (!ok) {
		Serial.println("No response or invalid response!");
	}
	return ok;
}

bool sendRequest(int deviceId, double statusValue) {
    String stringValue = String(statusValue);
    stringValue.replace(".", ",");
    String cmdPath = String("/JSON?request=controldevicebyvalue&ref=") + deviceId + "&value=" + stringValue;

    Serial.print("Requesting URL: ");
    Serial.println(cmdPath);

    client.print(String("GET ") + cmdPath + " HTTP/1.0\r\n" + "Connection: close\r\n\r\n");
    return true;
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
            updateHomeSeer(reservoirWaterInletId, 100);
        }
        else
        {
            digitalWrite(alarmNoFlowPin, LOW);
            updateHomeSeer(reservoirWaterInletId, 0);
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
        Serial.print("Flow rate: ");
        Serial.print(int(flowRate));  // Print the integer part of the variable
        Serial.print(" L/min");
        Serial.print("\t"); 		  // Print tab space
        updateHomeSeer(flowRateId, (int)flowRate);
        // Print the cumulative total of litres flowed since starting
        Serial.print("Output Liquid Quantity: ");        
        Serial.print(totalMilliLitres);
        Serial.println(" mL"); 
        Serial.print("\t"); 		  // Print tab space
        Serial.print(totalMilliLitres/1000);
        Serial.print(" L");
        updateHomeSeer(totalVolumeId, (int)totalMilliLitres/1000);

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