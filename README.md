# ottis
This repository contains our code for a modified Ottis robot.

Ottis robot has been developed by Eduardo. There is a instructables page:
https://www.instructables.com/Ottis-Robot-the-Arduino-Robot-That-Can-Walk-and-Pe
and a neat YT video: https://www.youtube.com/watch?v=6Kt6kMrS9pQ

We have also modified the body - mostly head, since we use different electronics:

Arduino nano strong, which is very convenient for connecting servos
and Arduino nano for second controller responsible for controlling display and
other electronics. We use a pair of 14500 rechargable batteries and 4A power supply,
you may use some stronger LiPo instead - just replace it for the battery box.


List of parts:

mechanical:

* original parts of Eduardo at instructables page
* modified head https://www.tinkercad.com/things/3jyP8LSEyXB-ottis-new-head-for-nano-strong

electronics:

* Arduino nano strong
* Arduino nano
* 1.8" 128x160 RGB TFT LCD (Adafruit ST7735 compatible)
* BT module (HC-05 compatible, we use JDY-31)
* DfPlayer Mini
* 2W speaker 40mm diameter
* 2x 3mm LEDs and 2x 150 Ohm resistors
* 11x servo SG-90
* HC-SR04 ultrasonic
* MPU6050 gyro (optional)
* 2x capcitive touch sensor
* passive buzzer
* plastic holder for 2 AA-size batteries with strong leads
* 2x LiIon 14500 batteries
* KCD11 switch
* 4A5V power module, and 1:10 resistor power divider, such as 22k and 220k for measuring battery voltage
* connecting wires


Pin connections:

Main board (arduino nano strong, using pins and dupont cables):

*  0,1 - used for USB programming and serial communication (left free)
*    2 - Rx (BT Tx) - software serial input direction from BT module
*    3 - Servo
*    4 - Tx (BT Rx) - software serial output direction to BT module
* 5-12 - Servo
*   13 - passive buzzer
* A0/A1 (14,15)  - Servo
* A2   - US TRIG
* A3   - US ECHO
* A4   - Tx (to second arduino)
* A5   - Rx (from second arduino)
* A6   - battery measure from voltage divider (against 1.1V internal)


Second board (arduino nano, connections soldered, display on detachable connectors)

*  0,1 - used for USB programming and serial debugging communication (left free)
*    2 - Rx (from first arduino)
*    3 - Tx (to first arduino)
*    4 - DfPlayer Mini Rx serial pin
*    5 - LED1
*    6 - LED2
*    8 - TFT DC
*    9 - TFT RST
*   10 - TFT CS
*   11 - TFT MOSI
*   13 - TFT SCLK
*   A2 - touch1
*   A3 - touch2
*   A4 - gyro SDA (optional)
*   A5 - gyro SCL (optional)


Software requirements:

* regular arduino IDE with usual libraries:
* Arduino_mpu6050_master
* Adafruit_ST7735 (has two more Adafruit prereq) 
* for our TFT, this required modifying file   Adafruit_ST7735_and_ST7789_Library\Adafruit_ST7735.cpp as follows:
 insert lines:

    _colstart = 2;

    _rowstart = 1;

 behind line:

    displayInit(Rcmd2red);



Building:

* make sure you calibrate the servos to 90 degree position when mounting to a middle range position


Control:

* connect over USB or BT (even from mobile phone using arduino serial controller) and type 'h' to get help
* three modes: Control mode (direct control), Edit mode (editting choreographies), Debug mode (debugging choreographies)
* you may want to change the initial[] array 


Development:

* early trying after all pieces connected: https://www.youtube.com/watch?v=y5BnbnsZND8
