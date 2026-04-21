// Control motor speed using GY521
// https://www.circuitstoday.com/dc-motor-speed-control-using-gy521 

#include <Wire.h>
#include <MPU6050.h>

#define motor1_pin1 5
#define motor1_pin2 6
#define motor2_pin1 9
#define motor2_pin2 10

MPU6050 gy_521;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int motor1_speed;
int motor2_speed;

void setup ( )
 {
Wire.begin( );

Serial.begin (9600);

Serial.println ("Initializing MPU and testing connections");

gy_521.initialize ( );

Serial.println(gy_521.testConnection( ) ? "Successfully Connected" : "Connection failed");

delay(1000);

Serial.println("Reading Values");

delay(1000)
}

void loop ( )
{
gy_521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
ax = map(ax, -17000, 17000, -125, 125);

motor1_speed = 125+ax;    //To move first motor
motor2_speed = 125-ax;    //To move second motor

Serial.print ("Motor1 Speed = ");
Serial.print (motor1_speed, DEC);
Serial.print (" && ");
Serial.print ("Motor2 Speed = ");
Serial.println (motor2_speed, DEC);

analogWrite (motor1_pin2, motor1_speed);
analogWrite (motor2_pin2, motor2_speed);

delay (200);
}




// Control motor speed using weight
Vector3D tlWeight(0, 0, 250);  // Weight vector for top-left motor
Vector3D trWeight(0, 0, 250);  // Weight vector for top-right motor
Vector3D blWeight(0, 0, 250);  // Weight vector for bottom-left motor
Vector3D brWeight(0, 0, 250);  // Weight vector for bottom-right motor


// Constants for motor thrust and gravity
const float motorThrust = 100.0;
const float gravity = 9.81;      


void setup() {
  // Assigning different ESCs to different ports
  tlESC.attach(9);
  trESC.attach(10);
  blESC.attach(11);
  brESC.attach(12);
}


void loop() {
  // Calculate total weight vector
  Vector3D totalWeight = tlWeight + trWeight + blWeight + brWeight;


  // Calculate desired thrust for each motor to counteract gravity
  Vector3D thrust = totalWeight * gravity;


  // Calculate speed adjustment for each motor based on thrust
  int tlSpeedAdjustment = map(thrust.magnitude(), 0, motorThrust, -500, 500);
  int trSpeedAdjustment = map(thrust.magnitude(), 0, motorThrust, -500, 500);
  int blSpeedAdjustment = map(thrust.magnitude(), 0, motorThrust, -500, 500);
  int brSpeedAdjustment = map(thrust.magnitude(), 0, motorThrust, -500, 500);




#include <ArduinoBLE.h>
#include <Servo.h>


// Define ESC pins
const int escPins[] = {5,6,9.10};


// Number of ESCs
const int numEscs = 4;


// Minimum and maximum throttle values
const int minThrottle = 1000;
const int maxThrottle = 2000;


// Servo objects for each ESC
Servo escs[numEscs];


// Define BLE Service and Characteristic UUIDs
BLEService throttleService("19B10010-E8F2-537E-4F6C-D104768A1214"); // Throttle Service UUID
BLEIntCharacteristic throttleCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Throttle Characteristic UUID


void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);


  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");


  // Set up BLE Service and Characteristic
  BLE.setLocalName("ESC Controller");
  BLE.setAdvertisedService(throttleService);
  throttleService.addCharacteristic(throttleCharacteristic);
  BLE.addService(throttleService);
  throttleCharacteristic.setValue(0);


  // Start advertising
  BLE.advertise();


  // Attach ESCs to pins
  for (int i = 0; i < numEscs; i++) {
    escs[i].attach(escPins[i]);
  }


  // Wait for ESCs to initialize
  delay(2000);
}


void loop() {
  // Check for BLE connections
  BLEDevice central = BLE.central();


  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());


    // While the central is still connected, read throttle commands
    while (central.connected()) {
      // Read throttle value from BLE characteristic
      int throttleValue = throttleCharacteristic.value();


      // Map throttle value to ESC pulse width
      int throttle = map(throttleValue, 0, 1023, minThrottle, maxThrottle);


      // Set throttle for each ESC
      for (int i = 0; i < numEscs; i++) {
        escs[i].writeMicroseconds(throttle);
      }


      // Print throttle value for debugging
      Serial.print("Throttle: ");
      Serial.println(throttleValue);


      // Delay between updates
      delay(100);
    }


    // When the central disconnects, stop the ESCs
    for (int i = 0; i < numEscs; i++) {
      escs[i].writeMicroseconds(1000); // Stop signal
    }
    Serial.println("Disconnected");
  }
}
