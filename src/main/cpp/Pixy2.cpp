// YourLink example class declaration 
// 
// This is an example class for use in the template class TPixy2
// (https://github.com/charmedlabs/pixy2/blob/master/src/host/arduino/libraries/Pixy2/TPixy2.h)
//
// Refer to 
// https://github.com/charmedlabs/pixy2/blob/master/src/host/arduino/libraries/Pixy2/Pixy2UART.h 
// or 
// https://github.com/charmedlabs/pixy2/blob/master/src/host/arduino/libraries/Pixy2/Pixy2.h 
// for working examples of template classes that work with TPixy2. 
#define NULL __null

#include <stdint.h>
#include "TPixy2.h"
#include "Pixy2CCC.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"

class YourLink
{

public:
  // open() is called upon initialization.  A 32-bit arg is passed as-is from the TPixy2 init() function.  
  // You can ignore the arg or use it to set up an address, data rate, whatever you want. 
  // Returns 0 if successful, or <0 if it fails
  int8_t open(uint32_t arg);
  
  // close is called when the TPixy2 instance is destroyed. 
  void close();
  
  // recv() takes a pointer to a data buffer where the received data will be written/returned and 
  // the number of bytes to receive via your serial port (SPI, I2C or UART).  It returns the number of 
  // bytes immediately available and written into the buffer (without needing to busy-wait.)  
  // It also takes a pointer to a 16-bit unsigned int that it will write the simple checksum of all 
  // bytes received, but only if the pointer is non-null.  Refer to the example classes in the links above.
  int16_t recv(uint8_t *data, uint8_t len, uint16_t *checksumCalculation=NULL);
  
  // send() takes a pointer to the data you want to send and the number of bytes to send via your serial 
  // port (SPI, I2C or UART).  It returns the number of bytes successfully sent.  
  int16_t send(uint8_t *data, uint8_t len);
  
  // ...
};


// These two functions are borrowed from the Arduino API and need to be implemented in some form 
// in order for the existing TPixy2 code to work as-is.  millis() may be difficult to implement, 
// but its use isn't hugely important.  Look in TPixy2.h for its references, and remove if necessary.  
// delayMicroseconds() can be easily implemented by a simple nested for-loop, and calibrated. 
uint32_t millis();
void delayMicroseconds(uint32_t us);


// Define an easy-to-use type for Pixy2 that uses your link as the class parameter to TPixy2
typedef TPixy2<YourLink> Pixy2WithYourLink;