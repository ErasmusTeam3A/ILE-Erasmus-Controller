#include <ArduinoBLE.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

BLEService imuService("0000ffb0-0000-1000-8000-00805f9b34fb"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic imuCharacteristic("00002AB3-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 12);

long previousMillis = 0;  // last timechecked, in ms

const int ledPin = LED_BUILTIN; // pin to use for the LED

//VARIABLES FOR EMU SENSOR

//define I2C channel
#define MPU 0x68
 
//conversion ratios
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//conversion of radians to degrees 180 / PI
#define RAD_A_DEG = 57.295779


int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

long pre_time;
float dt;
  

void setup() {
  //setup sensor
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // setup bluetooth
  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService); 
  imuService.addCharacteristic(imuCharacteristic);
  BLE.addService(imuService); 
 
  // start advertising
  BLE.advertise();

  Serial.println("BLE started");

}

void sendSensorData() {

Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Request register 0x3B - corresponds to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //From 0x3B, 6 registers are requested
  AcX=Wire.read()<<8|Wire.read(); //Each value occupies 2 registers
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
 
  //Y, X angles are calculated based on the accelerometer values
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
  //Read Gyro values
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //6 registrations are requested from 0x43
  GyX=Wire.read()<<8|Wire.read(); //Each value consists of 2 registers
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
 
  // Calculate the angles of Gyro
  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;
  Gy[2] = GyZ/G_R;
    
  dt = (millis() - pre_time) / 1000.0;
  pre_time = millis();
    
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1]; 

  //Calculate YAW
  Angle[2] = Angle[2]+Gy[2]*dt;


// Send 3x eulers over bluetooth as 1x byte array 
imuCharacteristic.setValue((byte *) &Angle, 12); 
    
      Serial.print("gX = "); Serial.print(Angle[0]);
      Serial.print("  gY = "); Serial.print(Angle[1]);
      Serial.print("  gZ = "); Serial.print(Angle[2]);

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
          sendSensorData();
          Serial.println();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  
}
