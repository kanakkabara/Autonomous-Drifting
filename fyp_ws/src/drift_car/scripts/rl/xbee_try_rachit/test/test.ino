#include <Servo.h>
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Wire.h>  
#include <XBee.h>
#include <stdio.h>
#include "pb_encode.h"
#include "pb_decode.h"
#include "messages.pb.h"
XBee xbee = XBee();
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4108214A);
TxStatusResponse txStatus = TxStatusResponse();
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  xbee.setSerial(Serial);

}

void loop() {
  
    uint8_t payload[1024];
    bool status;    
    fyp_Action action = {1,2.0};
   
    Serial.print("here");
    pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));
   
    status = pb_encode(&stream, fyp_Action_fields, &action);
//    payload[0] = 1;
//    payload[1] = 1;
//    payload[2] = 1;
      
    Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
    if (!status){
        Serial.println(PB_GET_ERROR(&stream));
        //return 1;
    }
    else{
      Serial.print("Sending");
      xbee.send(tx);  
    }

}
