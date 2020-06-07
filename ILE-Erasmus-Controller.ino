#include <ArduinoBLE.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

BLEService imuService("0000ffb0-0000-1000-8000-00805f9b34fb"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic imuCharacteristic("00002AB3-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 12);

long previousMillis = 0;  // last timechecked, in ms

const int ledPin = LED_BUILTIN; // pin to use for the LED

//VARIABLES FOR EMU SENSOR
// const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

// int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
// int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
// int16_t temperature; // variables for temperature data

//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

long tiempo_prev;
float dt;

// char tmp_str[7]; // temporary variable used in convert function

// char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
//   sprintf(tmp_str, "%6d", i);
//   return tmp_str;
// }
//END VARIABLES
  
//START LED BLE
void setup() {
  //START WIRE.H
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  //END WIRE.H

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

}

//SENSOR DATA FUNCTIE
void sendSensorData() {

Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
  AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
 
  //Y-, X-hoeken worden berekend op basis van de waarden van de versnellingsmeter
  //respectievelijk met de tangensformule.
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
  //Lees de Gyroscoop-waarden
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //Vanaf 0x43 worden 6 registraties aangevraagd
  GyX=Wire.read()<<8|Wire.read(); //Elke waarde beslaat 2 registers
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
 
  // Bereken de hoek van de gyroscoop
  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;
  Gy[2] = GyZ/G_R;
    
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
    
  //Pas het complementaire filter toe
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1]; 

  //Integratie met betrekking tot tijd om de YAW te berekenen
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
//          if (IMU.accelerationAvailable()) { // XX
//        previousMillis = currentMillis;
          sendSensorData();
          // delay(1000);
                    
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
