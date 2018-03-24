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

#ifndef _PX4FLOW_H
#define _PX4FLOW_H

// 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2) 
#define PX4FLOW_ADDRESS 0x42

// timeout in milliseconds for PX4Flow read
#define PX4FLOW_TIMEOUT 10

// If set to true, will print error messages on Serial
#define PX4FLOW_DEBUG true

// As described in the documentation
// http://pixhawk.org/modules/px4flow
typedef struct i2c_frame
{
    uint16_t frame_count;// counts created I2C frames
    int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
    int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
    int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
    int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
    int16_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    int16_t gyro_x_rate; //gyro x rate
    int16_t gyro_y_rate; //gyro y rate
    int16_t gyro_z_rate; //gyro z rate
    uint8_t gyro_range; // gyro range
    uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} i2c_frame;

typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
} i2c_integral_frame;

class PX4Flow
{
  public:
    PX4Flow();
    bool update();
    bool update_integral();

    // Simple frame
    uint16_t frame_count();
    int16_t pixel_flow_x_sum();
    int16_t pixel_flow_y_sum();
    int16_t flow_comp_m_x();
    int16_t flow_comp_m_y();
    int16_t gyro_x_rate();
    int16_t gyro_y_rate();
    int16_t gyro_z_rate();
    int16_t qual();
    uint8_t sonar_timestamp();
    int16_t ground_distance();

    // Integral frame
    uint16_t frame_count_since_last_readout();
    int16_t pixel_flow_x_integral();
    int16_t pixel_flow_y_integral();
    int16_t gyro_x_rate_integral();
    int16_t gyro_y_rate_integral();
    int16_t gyro_z_rate_integral();
    uint32_t integration_timespan();
    uint32_t sonar_timestamp_integral();
    int16_t ground_distance_integral();
    int16_t gyro_temperature();
    uint8_t quality_integral();


  protected:
    struct i2c_frame frame;
    struct i2c_integral_frame iframe;
    uint32_t read32();
    uint16_t read16();
    uint8_t read8();
    bool wait(int count);
};

#endif
