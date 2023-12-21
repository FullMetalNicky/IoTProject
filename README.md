# Water Quality Monitoring System
The water quality monitoring system would use the information on the condition of the water
(temperature, radiation, PH levels, composition), as well as geo-location data to analyze the
quality of the water periodically. The system will issue a warning when water quality is poor,
based on user provided threshold values. The system would collect a water sample through
the actuator elements (pumps), and will transfer them back to the water source once the
analysis is done

## Demo

https://github.com/FullMetalNicky/IoTProject/assets/25340794/92b1e9a9-4a76-47d5-8e2c-b75113e646b5


## Installation 
The system was tested on Arduino Portenta H7, and may have compatibility issues with other hardware. 
Required Arduino packages:
* TinyGPS (by Mikal Hart)
* OneWireNg
* PERIPUMP
* DFRobot_PH
* DFRobot_Geiger
* SparkFun_AS726X

As DFRobot_Geiger cannot be found on the Arduino library manager, you can download the .zip file from [here](https://github.com/cdjq/DFRobot_Geiger), and manually include it in the project. 
