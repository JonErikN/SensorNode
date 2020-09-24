#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>
#include <WiFi.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

#define WIFI_SSID "AafoskaasSoer"
#define WIFI_PASS "super1979"
#define MQTT_SERVER "10.0.0.3" //10.0.0.3

#define MQTT_MAX_PACKET_SIZE 4096

const int xPin = 34; //A2, GY-61 x-axis connected (green)
const int zPin = 39; //A4, GY-61 z-axis connected (orange)
const int yPin = 36; //A3, GY-61 y-axis connected (yellow)

const int batPin = 35; //A13 Bsttery is connected internaly to this pin

int redLed = 14;
int greenLed = 14;

#define OneWire1 27
#define OneWire2 33

OneWire oneWire(OneWire1);
OneWire oneWire2(OneWire2);

DallasTemperature sensorsTemp1(&oneWire);
DallasTemperature sensorsTempMotor(&oneWire2);


WiFiClient espClient;
PubSubClient client(MQTT_SERVER, 1883, espClient);
PubSubClientTools mqtt(client);

const float vibrationDataPoints = 4096;//8192; //Max 32768 points. Json limit.
const long samplingFrequency = 5000; //Enter sampling frequency (Hz) here
const long interval = 1000000 / samplingFrequency; //In micro seconds

unsigned long timeStart;
unsigned long timeEnd;
unsigned long timeElapsted;
unsigned long previousMicros = 0;

unsigned long sensingStartTime;
unsigned long wiFiStartTime;

int sensingInterval = 5; //This time in [s] is combined awake time and sleep time

String s = " ";

int loopCounter = 0;

int sampleTime;
long long int totalLoopTime;

char bufferMotor[50000];
DynamicJsonDocument motor(33000); //Serial.println(ESP.getMaxAllocHeap());

char bufferDebug[1000];
DynamicJsonDocument debug(1000); //Serial.println(ESP.getMaxAllocHeap());

float rebootCounter = 0;




void setup()
{
    pinMode(greenLed, OUTPUT);
    //pinMode(redLed, OUTPUT);


    analogReadResolution(12); //Set analogRead resolution to maximum (range 9-12)
    analogSetCycles(50); //Minimum 10! //Set the number of samples to take. Result is average value (range 1-256)
    analogSetSamples(1); //Set the number sets of samples to be run. Result is the value times factor.

    sensorsTemp1.begin();
    sensorsTempMotor.begin();
    sensorsTemp1.setResolution(11); //9-12 bits. 12 is quite slow (500ms?)
    sensorsTempMotor.setResolution(11); //9-12 bits. 12 is quite slow (500ms?)



    Serial.begin(115200); //Start serial.
    Serial.println("Setup finnished");

    //mqtt.subscribe("test/#", topic1_subscriber);
}

void loop()
{
    //Serial.println("Main loop starting");
    sensingStartTime = micros();

    btStop();
    //WiFi.mode(WIFI_MODE_NULL);


    totalLoopTime = 0;
    float loopTime = totalLoopTime / vibrationDataPoints;


    motor["TAG"] = "12-153"; //Example

    sensorsTempMotor.requestTemperatures();
    float temperatureMotor = sensorsTempMotor.getTempCByIndex(0);
    sensorsTemp1.requestTemperatures();
    float temperatureAmbient = sensorsTemp1.getTempCByIndex(0);

    motor["tempMotor"] = temperatureMotor;
    motor["temperatureAmbient"] = temperatureAmbient;
    motor["temperatureDifference"] = (temperatureMotor - temperatureAmbient);
    motor["batteryVoltage"] = (analogRead(batPin)*1.73 / 1000);
    //motor["batteryVoltage"] = (analogRead(batPin) * 2)-0.676;
    rebootCounter++;
    motor["rebootCounter"] = rebootCounter;



    motor["avgDeltaTime[us]"] = interval;
    motor["RPM"] = 1200; //20Hz = 1200RPM
    //motor["RPM"] = 190 + ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 20);
    JsonArray dataX = motor.createNestedArray("x[G]");
    JsonArray dataY = motor.createNestedArray("y[G]");
    JsonArray dataZ = motor.createNestedArray("z[G]");
    Serial.println("Acc sensing starting");
    digitalWrite(greenLed, HIGH);
    while (loopCounter <= (vibrationDataPoints - 1))
    {
        unsigned long currentMicros = micros();
        if (currentMicros - previousMicros >= interval)
        {
            previousMicros = currentMicros;
            dataX.add(ReadAxis(1));
            dataY.add(ReadAxis(2));
            dataZ.add(ReadAxis(3));

            ++loopCounter;

            totalLoopTime = totalLoopTime + (micros() - currentMicros);
        }
    }
    digitalWrite(greenLed, LOW);
    Serial.println("Acc sensing finnished");
    motor["timeSleep"] = (sensingInterval * 1000000);
    debug["timeAnalogRead"] = totalLoopTime / vibrationDataPoints;
    debug["interval"] = interval;


    unsigned long sensingTime = (micros() - sensingStartTime);
    debug["timeSensing"] = sensingTime;
    int timeAwake = ((micros() - wiFiStartTime) + (sensingInterval * 1000000));
    motor["timeAwake"] = timeAwake;
    motor["timeWiFi"] = (micros() - wiFiStartTime);

    wiFiStartTime = micros();

    serializeJson(motor, bufferMotor);
    motor.clear();



    loopCounter = 0;
    Serial.println("WiFi starting");
   

    digitalWrite(greenLed, HIGH);

    connectToWifi();
    Serial.println("WiFi connected");
    Serial.println("Connecting to MQTT");
    connectToMqtt();
    client.publish_P("Inovyn/SensorNode001/Motor001", bufferMotor, false);

    debug["timeWiFi"] = (micros() - wiFiStartTime);
    serializeJson(debug, bufferDebug);
    debug.clear();

    client.publish_P("Inovyn/SensorNode001/Debug", bufferDebug, false);
    //publisher("Inovyn/SensorNode001/Motor001", bufferMotor);

    if (client.connect("ESP32Client"))
    {

        //mqtt.subscribe("test_in/foo/bar", topic1_subscriber);
    }

    digitalWrite(greenLed, LOW);
    Serial.println("Sleeping start");
    //delay(2000);
    //int sleepTime = (sensingInterval * 1000000) - timeAwake;
    //esp_sleep_enable_timer_wakeup(10* 1000000);
    //esp_sleep_enable_timer_wakeup(sensingInterval * 1000000);
    //Serial.flush();
    //esp_light_sleep_start();
    //esp_deep_sleep_start(); //Light sleep will retain RAM
}
//void topic1_subscriber(String topic, String message) {
//    Serial.println(s + "Message arrived in function 1 [" + topic + "] " + message);
//}
//void publisher(String topic, String message)
//{
//    const String s = "";
//    mqtt.publish(topic, s + message);
//    delay(1000);
//}
//void publisher(String topic, float message)
//{
//    const String s = "";
//    mqtt.publish(topic, s + message);
//    delay(1000);
//}


void connectToWifi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
    }
    delay(100);
}

void connectToMqtt()
{
    if (client.connect("ESP32Client", "engineer", "vykgVjYTPDcK")) { //"inovyn", "inovyn"))
    }
    else {
    }
    delay(100);
}
//void blink(int numberofblinks, int blinkduration)
//{
//    for (int i = 1; i <= numberofblinks; i++)
//    {
//        digitalwrite(led, high);
//        delay(blinkduration);
//        digitalwrite(led, low);
//        delay(blinkduration);
//    }
//}
float ReadAxis(int axis)
{
    //enum axis { x, y, z };
    float xValue; //Variable for storing raw data from xPin
    float zValue; //Variable for storing raw data from zPin
    float yValue; //Variable for storing raw data from yPin

    float xG; //Variable for storing calculated G-value on x-axis
    float zG; //Variable for storing calculated G-value on z-axis
    float yG; //Variable for storing calculated G-value on y-axis

    switch (axis)
    {
    case 1:
        xValue = analogRead(xPin); //Use analogRead to sample the value on xPin
        xG = (xValue - (1800 + 44)) / 368; //Subtract (1800+calibration) to get the base value to 0. Divide by 368 to get value in g. 1g = 368 or 300mV.
        return xG;
        break;
    case 2:
        yValue = analogRead(yPin); //Use analogRead to sample the value on xPin
        yG = (yValue - (1800 + 56)) / 368; //Subtract (1800+calibration) to get the base value to 0. Divide by 368 to get value in g. 1g = 368 or 300mV.
        return yG;
        break;
    case 3:
        zValue = analogRead(zPin); //Use analogRead to sample the value on xPin
        zG = (zValue - (1800 + 27)) / 368; //Subtract (1800+calibration) to get the base value to 0. Divide by 368 to get value in g. 1g = 368 or 300mV.
        return zG;
        break;
    }
}
