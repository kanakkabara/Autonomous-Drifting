#include <XBee.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h> 
#include "Utils.h"
#include "PX4Flow.h"

volatile int x = 0;
const int ESC_PIN = 9;
const int STEERING_SERVO_PIN = 5;
const int STEERING_ANGLE_STEP = 10;
const int THROTTLE_STEP = 1;
const int LENGTH_OF_RX_PACKET = 2;

bool status;
unsigned long start = millis();
uint8_t* data;
int numFloats = 5;
float* floatValues;
float x_error, y_error, z_error;
float gyroBias1[3], accelBias1[3], magCalibration1[3]={0,0,0};
float SelfTest1[6];
Servo esc;
Servo steeringServo;
XBee xbee = XBee();
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4108214A);
TxStatusResponse txStatus = TxStatusResponse();
float gThrottle=0;
float gSteering=0;
void setup() {
  Wire.begin();
  Serial.begin(57600);
  xbee.setSerial(Serial);
  esc.attach(ESC_PIN);
  steeringServo.attach(STEERING_SERVO_PIN);
  Serial.flush();

  MPU9250SelfTest(SelfTest1);
  calibrateMPU9250(gyroBias1, accelBias1);
  initMPU9250();
  initAK8963(magCalibration1);
 
  issueCommands(/*throttle=*/0, /*steeringAngle=*/90);
  averageAccelerationX();
  averageAccelerationY();
  //z_error = averageAccelerationZ();
  //calcNoise();
}

int option = 0;
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
float yaw_1=0;
float gz_1=0;
float roll_1=0;
float pitch_1=0;
float xVelocityImu = 0;
float yVelocityImu = 0;
float zVelocityImu = 0;
float xVelocity;
float yVelocity;
long last_check = 0;
int px = 0;
int py = 0;
void loop() {
    if (xbee.readPacket(200)) {
      // Got a rx packet.
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
        if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
          xbee.getResponse().getRx16Response(rx16);
          option = rx16.getOption();
          data = rx16.getData();
        } else {
          xbee.getResponse().getRx64Response(rx64);
          option = rx64.getOption();
          data = rx64.getData();
        }
        
        // Received RX packet. Issue the actions in message.
        parseAndExecuteActions(data, LENGTH_OF_RX_PACKET);
        // Communicate the current state.
        communicateState();
      }
    } else {
      // Communicate state even if action message not received within timeout.
      communicateState();
    }
}

// union to convery float to byte string
union floatContainer {
    uint8_t b[4];
    float fval;
} floatConverter;

// count = number of floats represented by
// data array.
float* readFloat(uint8_t data[], int count){
  float values [count];
  for(int i = 0; i < count; i++){
    for (int j = 0; j < 4; j++) {
      floatConverter.b[j] = data[i*4 + j];
    }
    values[i] = floatConverter.fval;
  }
  return values;
} 


// Issue the throttle and steering actions received in data message.
void parseAndExecuteActions(uint8_t* arr, int num_floats) {
  // Received RX packet. Extract the actions.
  float* convertedFloats = Utils::convert_to_floats(arr, num_floats);
  
  // Extract values 
  float throttle = convertedFloats[0];
  float steeringAngle = convertedFloats[1];
  gSteering = steeringAngle;
  gThrottle = throttle;
  throttle = constrain(throttle, 0, 120);
  float steeringDegrees = 90+(steeringAngle*180)/3.1428;
  steeringDegrees = constrain(steeringDegrees, 65, 115);

  // Execute the actions and free space.
  issueCommands(throttle, steeringDegrees);
  free(convertedFloats);
}


void issueCommands(float throttle, float steeringDegrees) {
  esc.write(throttle);
  steeringServo.write(steeringDegrees);
}

// Communicates the state of the car via XBee.
// State: xdot, ydot, thetadot.
void communicateState() {
  // Read the IMU data.
  Wire.beginTransmission(104);
  imu();
  yaw_1 = getYaw();
  gz_1 = getGz();
  roll_1 = getRoll();
  pitch_1 = getPitch();
  xVelocityImu = getVx();
  yVelocityImu = getVy();
  zVelocityImu = getVz();
  float ax = getAx();
  float ay = getAy();
  Wire.endTransmission();

  // Create payload to send.
  float toSend[] = {xVelocityImu, yVelocityImu, gz_1, gThrottle, gSteering};
  uint8_t* payload = Utils::convert_to_bytes(toSend, 8);
  
  // Send TX packet and free space.
  Tx64Request tx = Tx64Request(addr64, payload, numFloats * 4);
  xbee.send(tx);
  free(payload);
}


