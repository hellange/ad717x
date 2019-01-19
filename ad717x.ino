
int slaveSelectPin = 10;
#define ID 0x0CD9

unsigned int regValue;
#include <SPI.h>
#include "adc_setup.h"
#define FSR (((long int)1<<23)-1)

 long ret      = 0;
    long timeout  = 0x00FFFFFF;
    long sample   = 0;

void setup() {
  // put your setup code here, to run once:
   
   Serial.begin(9600);

   /*
   SPI.begin(); // initialize SPI, covering MOSI,MISO,SCK signals
   SPI.setBitOrder(LSBFIRST);  // data is clocked in MSB first
   SPI.setDataMode(SPI_MODE3);  // SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
   SPI.setClockDivider(SPI_CLOCK_DIV64);  // system clock = 16 MHz, chip max = 1 MHz
  */
   pinMode(10,OUTPUT);
   pinMode(11,OUTPUT);
   pinMode(12,INPUT);
   pinMode(13,OUTPUT);
 

      SPI.beginTransaction (SPISettings (8000000, MSBFIRST, SPI_MODE3)); // 100000

   //initADC7176();


       initADC7176();

    int32_t ret;

           ret = AD7176_WriteRegister(AD7176_regs[Interface_Mode_Register]);
    if(ret < 0)
        return ret;
    Serial.println(ret);

 

        

}
    float voltage = 0;


void loop()
{


      digitalWrite(slaveSelectPin,LOW);
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

            
    ret = AD7176_ReadRegister(&AD7176_regs[4]);
    Serial.print(ret);
    Serial.print(" bytes: ");
    Serial.print(AD7176_regs[4].value, HEX);
 
/*
    int8_t ready = 0;

    ret = AD7176_ReadRegister(&AD7176_regs[ID_st_reg]);
    Serial.println(AD7176_regs[ID_st_reg].value, BIN);
 
    ret = AD7176_ReadRegister(&AD7176_regs[IOCon_Register]);
    Serial.println(AD7176_regs[IOCon_Register].value, BIN);
*/


long vals[] ={4529955,
4865327,
5200545,
5540552,
5873235,
6208504,
6543996,
6879561,
7215237
};
//for (int i=0;i<9;i++) {
//  Serial.println(vals[i+1] - vals[i]);
//}

 float VREF = 5.0;             
 float VFSR = VREF; 
  
   float v = (float) ((AD7176_regs[4].value*VFSR*1000.0)/FSR); 
 v=v*4;
 v=v/0.8;
 v=v-12500.0;
v=v/2.0;
Serial.print(" raw ");
    Serial.print(AD7176_regs[4].value, BIN);
    
    Serial.print(" raw volt ");
    Serial.print(v);
    Serial.print(" adjusted gain ");

     v=v - v*0.00044;

    Serial.print(v);

  Serial.print(" adjusted offset ");

 float offset = 1.03;
 v=v-offset;
    Serial.println(v);
    delay(1000);
   }
}


