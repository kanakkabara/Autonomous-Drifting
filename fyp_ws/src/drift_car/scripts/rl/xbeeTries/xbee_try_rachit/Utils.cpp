#include "Utils.h"

namespace Utils {
  union DataConverter {
    float fval;
    uint8_t bytes[4];
  } converter;

  /*
    Converts an array of floats to an array of bytes.
  */
  uint8_t* convert_to_bytes(float* arr, int num_floats){
    uint8_t* converted_bytes = (uint8_t*) malloc(num_floats * 4);
    for (int i = 0; i < num_floats; i++){
      converter.fval = arr[i];
      memcpy(converted_bytes + 4*i, converter.bytes, 4);
    }
    return converted_bytes;
  }

  /*
    Converts an array of bytes to corresponding
    array of floats.
  */
  float* convert_to_floats(uint8_t* arr, int num_floats){
    float* converted_floats = (float*) malloc(num_floats * sizeof(float));
    for (int i = 0; i < num_floats; i++){
      memcpy(converter.bytes, arr + 4*i, 4);
      converted_floats[i] = converter.fval;
    }
    return converted_floats;
  }


  void print_floats_serial(float* arr, int arr_len) {
    Serial.print("\nFloat array: ");
    for (int i = 0; i < arr_len; i++) {
      Serial.print(arr[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}