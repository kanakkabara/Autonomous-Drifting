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

#include <Wire.h>
#include "PX4Flow.h"

#define PARAM_FOCAL_LENGTH_MM 3.6

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
#define LED 13

long last_check = 0;
float px = 0;
float py = 0;
float focal_length_px = (PARAM_FOCAL_LENGTH_MM) / (4.0f * 6.0f) * 1000.0f;
  
// Initialize PX4Flow library
PX4Flow sensor = PX4Flow(); 

float x_rate =0;
float y_rate =0;
float flow_x=0;
float flow_y=0;
float velocity_x;
float velocity_y;
float x_noise=0.0;
float y_noise=0.0;

void calcNoise(){
  int counter=0;
  x_noise = 0.0;
  y_noise = 0.0;
  long starting = millis();
  long ending = starting+5000;
  Serial.println("Calculating Noise");
  while(counter<3000){
    sensor.update_integral();
    int quality = sensor.quality_integral();
    if(quality>100){
      counter++;
      
      x_rate = sensor.gyro_x_rate_integral() / 10.0f;       // mrad
      y_rate = sensor.gyro_y_rate_integral() / 10.0f;       // mrad
      flow_x = sensor.pixel_flow_x_integral() / 10.0f;      // mrad
      flow_y = sensor.pixel_flow_y_integral() / 10.0f;      // mrad  
      int timespan = sensor.integration_timespan();               // microseconds
      int ground_distance = sensor.ground_distance_integral();
      float pixel_x = flow_x + x_rate; // mrad
      float pixel_y = flow_y + y_rate; // mrad
      velocity_x = pixel_x * ground_distance / timespan; // m/s
      velocity_y = pixel_y * ground_distance / timespan; 
      
      velocity_x = abs(velocity_x);
      velocity_y = abs(velocity_y);
      
      x_noise = ((x_noise*(counter-1))+velocity_x)/counter;
      y_noise = ((y_noise*(counter-1))+velocity_y)/counter;
    }
  }
  Serial.println("Noise Calculated");
  Serial.println(x_noise);
  Serial.println(y_noise);
}
void Px4Flow()
{
  long loop_start = millis();
  if (loop_start - last_check > 100) {
    // Fetch I2C data  
    sensor.update_integral();
    x_rate = sensor.gyro_x_rate_integral() / 10.0f;       // mrad
    y_rate = sensor.gyro_y_rate_integral() / 10.0f;       // mrad
    flow_x = sensor.pixel_flow_x_integral() / 10.0f;      // mrad
    flow_y = sensor.pixel_flow_y_integral() / 10.0f;      // mrad  
    int timespan = sensor.integration_timespan();               // microseconds
    int ground_distance = sensor.ground_distance_integral();    // mm
    int quality = sensor.quality_integral();
    
    if (quality > 100)
    {
      float pixel_x = flow_x + x_rate; // mrad
      float pixel_y = flow_y + y_rate; // mrad

      velocity_x = pixel_x * ground_distance / timespan;     // m/s
      velocity_y = pixel_y * ground_distance / timespan;     // m/s 
      // Scale based on ground distance and compute speed
      // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
      velocity_x -= x_noise;
      velocity_y -= y_noise;

      if(velocity_x<0) velocity_x=0;
      if(velocity_y<0) velocity_y=0;
      
      // Integrate velocity to get pose estimate
      px = px + velocity_x * 100;
      py = py + velocity_y * 100;

      last_check = loop_start;
   }
  }
}
float getXPosition(){
  return px;
}
float getYPosition(){
  return py;
}
float getXVelocity(){
  return velocity_x;
}
float getYVelocity(){
  return velocity_y;
}

