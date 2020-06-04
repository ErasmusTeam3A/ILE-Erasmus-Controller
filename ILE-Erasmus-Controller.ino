#include <ArduinoBLE.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

BLEService imuService("0000ffb0-0000-1000-8000-00805f9b34fb"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic imuCharacteristic("00002AB3-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 12);

long previousMillis = 0;  // last timechecked, in ms

const int ledPin = LED_BUILTIN; // pin to use for the LED

//VARIABLES FOR EMU SENSOR
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
//END VARIABLES
  
//START LED BLE
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // Setup bluetooth
  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService); 
  imuService.addCharacteristic(imuCharacteristic);
  BLE.addService(imuService); 
 
  // start advertising
  BLE.advertise();

  Serial.println("BLE started");

//END LED BLE

//START WIRE.H
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

//SENSOR DATA FUNCTIE
void sendSensorData() {
float eulers[3];


Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
// read orientation x, y and z eulers
//IMU.readEulerAngles(eulers[0], eulers[1], eulers[2]); 
//  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
//  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
//  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
//  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  eulers[0] = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  eulers[1] = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  eulers[2] = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

//eulers[0] = convert_int16_to_str(data[0]);
//eulers[1] = convert_int16_to_str(data[1]);
//eulers[2] = convert_int16_to_str(data[2]);

// Send 3x eulers over bluetooth as 1x byte array 
imuCharacteristic.setValue((byte *) &eulers, 12); 
    
      Serial.print("gX = "); Serial.print(eulers[0]);
      Serial.print("  gY = "); Serial.print(eulers[1]);
      Serial.print("  gZ = "); Serial.print(eulers[2]);

} 


void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a BLE central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      
      if (currentMillis - previousMillis >= 50) {
//          if (IMU.accelerationAvailable()) { // XX
//        previousMillis = currentMillis;
          sendSensorData();
                    
          Serial.println();

//          }
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  
}
