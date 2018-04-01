/*
 * Copyright (c) 2014 by Laurent Eschenauer <laurent@eschenauer.be>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include "PX4Flow.h":

PX4Flow::PX4Flow()
{
}

bool PX4Flow::update()
{
  //send 0x0 to PX4FLOW module and receive back 22 Bytes data 
  Wire.beginTransmission(PX4FLOW_ADDRESS);
  Wire.write(0x0);  
  Wire.endTransmission();  
  
  // request 22 bytes from the module
  Wire.requestFrom(PX4FLOW_ADDRESS, 22);    

  // wait for all data to be available
  if (!wait(22)) {
      return false;
  }

  // read the data
  frame.frame_count       = read16();
  frame.pixel_flow_x_sum  = read16();
  frame.pixel_flow_y_sum  = read16();
  frame.flow_comp_m_x     = read16();
  frame.flow_comp_m_y     = read16();
  frame.qual              = read16();
  frame.gyro_x_rate       = read16();
  frame.gyro_y_rate       = read16();
  frame.gyro_z_rate       = read16();
  frame.gyro_range        = read8();
  frame.sonar_timestamp   = read8();
  frame.ground_distance   = read16();
  
  // if too many bytes are available, we drain in order to be synched
  // on next read
  if(Wire.available()) {
    #if PX4FLOW_DEBUG == true
    {
      Serial.println("ERROR [PX4Flow] : Too many bytes available.");
    }
    #endif
    while(Wire.available()) {Wire.read();}
  }

  return true;
}

bool PX4Flow::update_integral()
{
  //send 0x16 to PX4FLOW module and receive back 25 Bytes data 
  Wire.beginTransmission(PX4FLOW_ADDRESS);
  Wire.write(0x16);  
  Wire.endTransmission();  
  
  // request 25 bytes from the module
  Wire.requestFrom(PX4FLOW_ADDRESS, 26);    

  // wait for all data to be available
  // TODO we could manage a timeout in order not to block
  // the loop when no component is connected
  if (!wait(26)) {
      return false;
  }
  
  // read the data
  iframe.frame_count_since_last_readout = read16();
  iframe.pixel_flow_x_integral  = read16();
  iframe.pixel_flow_y_integral  = read16();
  iframe.gyro_x_rate_integral   = read16();
  iframe.gyro_y_rate_integral   = read16();
  iframe.gyro_z_rate_integral   = read16();
  iframe.integration_timespan   = read32();
  iframe.sonar_timestamp        = read32();
  iframe.ground_distance        = read16();
  iframe.gyro_temperature       = read16();
  iframe.quality                = read8();

  // This is due to the lack of structure packing
  // in the PX4Flow code.
  read8();
  
  // if too many bytes are available, we drain in order to be synched
  // on next read
  if(Wire.available()) {
    #if PX4FLOW_DEBUG == true
    {
      Serial.println("ERROR [PX4Flow] : Too many bytes available.");
    }
    #endif
    while(Wire.available()) {Wire.read();}
  }

  return true;
}

// Simple frame
uint16_t PX4Flow::frame_count() {
  return frame.frame_count;
}

int16_t PX4Flow::pixel_flow_x_sum() {
  return frame.pixel_flow_x_sum;
}

int16_t PX4Flow::pixel_flow_y_sum() {
  return frame.pixel_flow_y_sum;
}

int16_t PX4Flow::flow_comp_m_x() {
  return frame.flow_comp_m_x;
}

int16_t PX4Flow::flow_comp_m_y() {
  return frame.flow_comp_m_y;
}

int16_t PX4Flow::gyro_x_rate() {
  return frame.gyro_x_rate;
}

int16_t PX4Flow::gyro_y_rate() {
  return frame.gyro_y_rate;
}

int16_t PX4Flow::gyro_z_rate() {
  return frame.gyro_z_rate;
}

int16_t PX4Flow::qual() {
  return frame.qual;
}

uint8_t PX4Flow::sonar_timestamp() {
  return frame.sonar_timestamp;
}

int16_t PX4Flow::ground_distance() {
  return frame.ground_distance;
}

// Integral frame
uint16_t PX4Flow::frame_count_since_last_readout() {
  return iframe.frame_count_since_last_readout;
}

int16_t PX4Flow::pixel_flow_x_integral() {
  return iframe.pixel_flow_x_integral;
}

int16_t PX4Flow::pixel_flow_y_integral() {
  return iframe.pixel_flow_y_integral;
}

int16_t PX4Flow::gyro_x_rate_integral() {
  return iframe.gyro_x_rate_integral;
}

int16_t PX4Flow::gyro_y_rate_integral() {
  return iframe.gyro_y_rate_integral;
}

int16_t PX4Flow::gyro_z_rate_integral() {
  return iframe.gyro_z_rate_integral;
}

uint32_t PX4Flow::integration_timespan() {
  return iframe.integration_timespan;
}

uint32_t PX4Flow::sonar_timestamp_integral() {
  return iframe.sonar_timestamp;
}
      
int16_t PX4Flow::ground_distance_integral() {
  return iframe.ground_distance;
}

int16_t PX4Flow::gyro_temperature() {
  return iframe.gyro_temperature;
}

uint8_t PX4Flow::quality_integral() {
  return iframe.quality;
}

// Protected
uint32_t PX4Flow::read32() {
  return (uint32_t) read16() + (uint32_t) (read16() << 16);
}

uint16_t PX4Flow::read16() {
  return Wire.read() + (uint16_t) (Wire.read() << 8);
}

uint8_t PX4Flow::read8() {
  return Wire.read();
}

bool PX4Flow::wait(int count) {
  unsigned long now = millis();
  while(Wire.available() < count) {

    if ((millis() - now) > PX4FLOW_TIMEOUT) {
      #if PX4FLOW_DEBUG == true
        {
          Serial.println("ERROR [PX4Flow] : Timeout reading PX4_Flow.");
        }
      #endif
      return false;
    }
    delay(1);
  }
  return true;
}
