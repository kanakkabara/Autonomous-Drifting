#include <XBee.h>

/*
This example is for Series 1 XBee
Sends a TX16 or TX64 request with the value of analogRead(pin5) and checks the status response for success
Note: In my testing it took about 15 seconds for the XBee to start reporting success, so I've added a startup delay
*/

XBee xbee = XBee();

unsigned long start = millis();

// allocate two bytes for to hold a 10-bit analog reading
uint8_t payload[] = {0, 0, 0};

// with Series 1 you can use either 16-bit or 64-bit addressing

// 16-bit addressing: Enter address of remote XBee, typically the coordinator
//Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4108214A);
Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

// RX packet variables.
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t option = 0;
char* data;

int pin5 = 0;

int statusLed = 11;
int errorLed = 12;

bool sentTxPacket = false;

void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  Serial.begin(9600);
  xbee.setSerial(Serial);
}

void loop() {
   // start transmitting after a startup delay.  Note: this will rollover to 0 eventually so not best way to handle
    if (millis() - start > 15000 && !sentTxPacket) {
      // break down 10-bit reading into two bytes and place in payload
      // pin5 = analogRead(5);
      payload[0] = 1;
      payload[1] = 2;
      payload[2] = 3;
      
      xbee.send(tx);
      sentTxPacket = true;
    }
    // after sending a tx request, we expect a status response
    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
        // got a response!
        // should be a znet tx status              
      if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
         xbee.getResponse().getTxStatusResponse(txStatus);
        
         // get the delivery status, the fifth byte
           if (txStatus.getStatus() == SUCCESS) {
              // success.  time to celebrate
              flashLed(statusLed, 5, 50);
           } else {
              // the remote XBee did not receive our packet. is it powered on?
              //flashLed(errorLed, 3, 500);
           }
        } else if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
          // got a rx packet
          if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
            xbee.getResponse().getRx16Response(rx16);
            option = rx16.getOption();
            data = (char *) rx16.getData();
          } else {
            xbee.getResponse().getRx64Response(rx64);
            option = rx64.getOption();
            data = (char *) rx64.getData();
          }
          Serial.println("");
          Serial.print("Got data :");
          Serial.println(data);
        }
    } else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");  
      //nss.println(xbee.getResponse().getErrorCode());
      // or flash error led
    } else {
      // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
      flashLed(errorLed, 2, 50);
    }
    
    delay(1000);
}
