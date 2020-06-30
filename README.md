# ILE-Erasmus
The ILE Erasmus app is targeted at students who are studying the anatomy of the female body. This repository contains the Ccntroller software for the Arduino MKR 1010 WiFi

### Installation
- Clone the project.
- Open [ILE-Erasmus-Controller.ino](https://nodejs.org/) and upload it via the [Arduino IDE](https://www.arduino.cc/en/main/software).

### Hardware

This project uses the [Arduino MKR 1010 WiFi](https://store.arduino.cc/arduino-mkr-wifi-1010) and the MPU-6050 Gyro sensor. You can buy them directly at Arduino or any other hobby electronics shop. The Arduino supports using an external battery, but it will also work fine over USB.

Don't forget to update your Arduino MKR's [firmware](https://forum.arduino.cc/index.php?topic=579306.0) if needed!

## Wiring
Connect the sensor with 2 cables to the SDL and SDA port, also connect VCC and GND to the VCC and GND ports on the Arduino.

This is how your controller should look.
![alt text](https://github.com/ErasmusTeam3A/ILE-Erasmus-Controller/blob/master/images/IMG_3315.jpeg?raw=true)

### Usage
Connect the controller with our [deployment](https://erasmusteam3a.github.io/ILE-Erasmus-Deploy.github.io/#/). Or use the included Javascript with this repository to get the values of the sensor. Warning: It only works with browsers that support Bluetooth. We tested with Chrome.


### End result

Now you can use the controller to manipulate the 3D model in your browser.
![alt text](https://github.com/ErasmusTeam3A/ILE-Erasmus-Controller/blob/master/images/IMG_3259.jpeg?raw=true)

### Known errors
If the controller is connected to the web interface for a long time, the web page might slow down.

License
----

- Hayen
- Casper
- Lars
- Jaron
- Mio
- Hogeschool Rotterdam