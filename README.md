# SensorNode
As part of our bachelors thesis "Monitoring and data acquisition on motor vibrations using a sensor network" we created a battery powered and wireless node based on ESP32 with vibration and temperatur sensors.

The code is written in C++ using Visual Studio with Visual Micro for debugging and uploading code to the microprosessor.

The node does the following:
1. Collects a chosen number of vibration datapoints at chosen frequency.
2. Reads the temperatures from the temperature sensors.
3. Writes all the datapoints are stored in a json document.
4. Connects to chosen Wi-Fi access point.
5. Connect and transfer all stored data to a MQTT broker.
6. Disconnect from MQTT broker and Wi-Fi.
7. Light or deep sleep

To physically assemble this project you need:
1. ESP32 microcontroller
2. Breadboard 
3. 3 axis analog vibration sensor
4. Correct capacitors calculated with formula using frequency and internal resistance as parameters.
5. Digitral temperatur sensors that uses Dallas library.
6. Single cell li-ion battery.
