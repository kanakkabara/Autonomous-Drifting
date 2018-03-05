#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdlib.h>
#include "Arduino.h"

namespace Utils {
  uint8_t* convert_to_bytes(float* arr, int num_floats);
  float*   convert_to_floats(uint8_t* arr, int num_floats);
  void     print_floats_serial(float* arr, int arr_len);
}

#endif