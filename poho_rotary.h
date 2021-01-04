/*
 * Rotary encoder library for Arduino.
 */

// #ifndef pohorotary_h
// #define pohorotary_h

#include "Arduino.h"
// Enable this to emit codes twice per step.
//#define HALF_STEP

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

class PohoRotary
{
  public:
    PohoRotary();
    // Process pin(s)
    unsigned char process(int val1, int val2);
  private:
    unsigned char state;
};

// #endif
