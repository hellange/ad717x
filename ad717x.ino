
int slaveSelectPin = 10;
#define ID 0x0CD9

unsigned int regValue;
#include <SPI.h>
#include "adc_setup.h"

#define FSR (((long int)1<<23)-1)

void setup() {   
   Serial.begin(9600);

   pinMode(10,OUTPUT);
   pinMode(11,OUTPUT);
   pinMode(12,INPUT);
   pinMode(13,OUTPUT);
 
   //SPI.beginTransaction (SPISettings (8000000, MSBFIRST, SPI_MODE3)); // 100000
   initADC7176();
   delay(1000);
   Serial.println("Start measuring...");

}
    
    
void loop()
{
/* 
    SPI.beginTransaction (SPISettings (12000000, MSBFIRST, SPI_MODE3)); // 100000
    //digitalWrite(slaveSelectPin,LOW);
    delay(5);
    SPI.transfer(B01000111); // Bit 6 = 1 for read. Register 0x07 for ID register.
    //digitalWrite(slaveSelectPin,HIGH);
    //digitalWrite(slaveSelectPin,LOW);
    delay(5);
    byte b1 = SPI.transfer(0);
    byte b0 = SPI.transfer(0);
    byte b2 = SPI.transfer(0);
    delay(5);
    // digitalWrite(slaveSelectPin,HIGH);
    unsigned int result = b0 * 256 + b2;
    Serial.println(result, HEX);
*/ 
   
   long ret = AD7176_ReadRegister(&AD7176_regs[Status_Register]);
   if(ret < 0) {
   } else {
   
     float VREF = 2.5000;             
     float VFSR = VREF; 
  
     ret = AD7176_ReadRegister(&AD7176_regs[4]);
     float v = (float) ((AD7176_regs[4].value*VFSR*1000.0)/FSR); 
     /*   relevant for development board
     v=v*4;
     v=v/0.8;
     v=v-12500.0;
     v=v/2.0;
     */
  
     v=v-VREF*1000.0;
  
     Serial.print("raw ");
     Serial.print(AD7176_regs[4].value, HEX); 
     Serial.print("=");
     Serial.print(v);
     
     float offset = 0.2;
     v=v+offset;
     Serial.print(" adjusted offset ");
     Serial.print(v);
  
     float gain_factor = 0.075;
     v = v + v * gain_factor;
     Serial.print(" adjusted gain ");
     Serial.print(v);
     Serial.println(" mv");
     delay(10);
   }
}
