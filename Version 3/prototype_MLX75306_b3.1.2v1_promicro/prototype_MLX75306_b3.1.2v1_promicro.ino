/*
Prototype for MLX75306 on Sparfun Pro Micro (32U4) 16 MHz 5v
Also for custom board v3.1.2  - Not for prior versions as pins have changed!

Features:
* USB-Serial ASCII output of measurement in micrometers
* Read the measurment over the I2C interface
* Uses high speed Timer 4 for higher frequency PWM for the LED (~90KhZ)


 */


 /*
Arduino Pins to Ports
Arduino Pin:0- PortBit: D2
Arduino Pin:1- PortBit: D3
Arduino Pin:2- PortBit: D1
Arduino Pin:3- PortBit: D0
Arduino Pin:4- PortBit: D4
Arduino Pin:5- PortBit: C6
Arduino Pin:6- PortBit: D7
Arduino Pin:7- PortBit: E6
Arduino Pin:8- PortBit: B4
Arduino Pin:9- PortBit: B5
Arduino Pin:10- PortBit: B6
Arduino Pin:11- PortBit: B7
Arduino Pin:12- PortBit: D6
Arduino Pin:13- PortBit: C7
Arduino Pin:14- PortBit: B3
Arduino Pin:15- PortBit: B1
Arduino Pin:16- PortBit: B2
Arduino Pin:17- PortBit: B0
Arduino Pin:18- PortBit: F7
Arduino Pin:19- PortBit: F6
Arduino Pin:20- PortBit: F5
Arduino Pin:21- PortBit: F4
Arduino Pin:22- PortBit: F1
Arduino Pin:23- PortBit: F0
Arduino Pin:24- PortBit: D4
Arduino Pin:25- PortBit: D7
Arduino Pin:26- PortBit: B4
Arduino Pin:27- PortBit: B5
Arduino Pin:28- PortBit: B6
Arduino Pin:29- PortBit: D6
Arduino Pin:30- PortBit: D5



  */

#include "mlx75306.h"
#include <EEPROM-Storage.h>
#include <SPI.h>
#include <TimerOne.h>
#include <FixedPoints.h>
#include <FixedPointsCommon.h>

// The best library for I2C since it supports full slave capabilities like an IC does
#include <HardWire.h>  




//#define DISPLAY_LED_PIN 17
//For the 3.1.2 board, change to PD7
#define DISPLAY_LED_PIN 6

//#define FR_PIN 6
//For the 3.1.2 board, change to PF4
#define FR_PIN 21


#define CS_PIN 7





#define OUTPUT_PWM_PIN 9
#define OUTPUT_PWM OCR1A

#define LED_PWM_PIN 10
#define LED_PWM OCR4B

#define BUTTON_PIN 5

#define LED_SW1_PIN 8
//#define LED_SW2_PIN 4
//For the 3.1.2 board, change to this (PD5)
#define LED_SW2_PIN 30

//#define POW_EN_PIN 21
//For the 3.1.2 board, change to this (PB0)
#define POW_EN_PIN 17



#define DA_PERIOD_TICKS 1000  //Number of ticks to use for the digital to analog output.  Since supply is 5v, 1 tick is 5 millivolts
#define LED_PERIOD_TICKS 1023 // Number of ticks to use for the LED digital to analog output
#define LED_SHIFT_FACTOR 2 //  Crude adjustment of gain for the LED illumination control
#define MIN_PIXEL_LIMIT 20 //defines the minimum width cutoff in pixels (20 pixels = 1 mm).  Sensor values below this value will be output as 0

//These defines set the two types of calibration rods that are possible to calibrate the sensor with
#define CALIB_STANDARD_A 31.75 //calibrate using a drill rod of .0625 in diameter (1.5875 mm)  set to (1.5875 mm)*(20 pixels/mm)=31.75
#define CALIB_STANDARD_B 60.0 //calibrate using a drill rod of 3mm   set to (3.00 mm)*(20 pixels/mm)=60
#define CALIB_STANDARD_THRESHOLD_B 36 // Threshold for automatically determining the size of the calibration rod based on inherent sensor accuracy.
#define CALIB_STANDARD_THRESHOLD_A 10 // Threshold for automatically determining the size of the calibration rod based on inherent sensor accuracy. 

#define PWM_BIAS 2 // Bias expected in output amplifier is 10mv, that means 2 counts out of 1000
#define WIDTH_FILTER_PARAM 4 //EWMA filter parameter for width.  4 indicates 4/256 or 0.015625
#define BANDGAP_FILTER_PARAM 4 //EWMA filter parameter for voltage bandgap measurement.  4 indicates 4/256 or 0.015625 

#define SHUTTER_DELAY_US 1000  //Shutter delay in microseconds for the linescan camera

#define LOOP_TIME 5000//loop time in microseconds


/********** EEPROM Storage  ********/
#define EEPROM_START 0
EEPROMStorage<byte> eeprom_meas_type(EEPROM_START, 0);  //set to default value of 0, uses 2 bytes including checksum
#define EEPROM_NEXT EEPROM_START + 2
EEPROMStorage<SQ7x8> eeprom_width_calib_fact(EEPROM_NEXT, 0.0);  //set to default value of 0.0, uses 3 bytes including checksum
/********** EEPROM Storage  ********/


/* Slave I2C Memory Registers */

// Need way to recognize end of communication with master so that the i2c_address can be set to NULL_ADDRESS !

typedef struct sensorData_t{

  
  volatile int width_micrometers; //width in micrometers at register 1
  volatile int lightstrength; //indicates illuminating strength of LED at register 3
  //This is expandable by adding other ints or bytes.  Put bytes (if any) last, since int alignment not an issue
  volatile byte reserved; // don't use last location - resting point for address pointer - indicates access complete
};

/* Alternate way to access same I2C memory registers*/
typedef union I2C_Packet_t{
sensorData_t value;
volatile byte I2CArray[sizeof(sensorData_t)];
};

#define PACKET_SIZE sizeof(sensorData_t)
#define NULL_ADDRESS sizeof(sensorData_t)-1

I2C_Packet_t iic_data; 

//examples for accessing I2C in code
//  iic_data.value.width_micrometers
//  iic_data.I2CArray[1]

byte i2c_address = NULL_ADDRESS;  //address of I2C
byte first_byte = 1; //flag indicating first byte of transmission

byte line_image[160];
char grey[11] =" .:-=+*#%@";

byte meas_type;  //indicates type of measurement, 0=ratiometric, >0=absolute
byte button_press;  //used to hold in ICSC program mode
byte count; //used to flash LED in ICSC program mode

byte keypress = 0;  //used to track previous keypress
byte timer = 0;  //counts up

long width_filter_storage =0;
long bandgap_filter_storage = 0;

unsigned long lastMicros=0, loop_time=0;

int lightstrength=(LED_PERIOD_TICKS/2) << LED_SHIFT_FACTOR; //strength of LED 0-1023, shifted by 2 to get more filtering


byte pct;  //keeps track of clock pulse count
word maxpixel,maxpixelfinal;  //indicate maximum light value (for detecting over-exposure)
 
word lastpixel;
word maxstep,minstep;  //tracks largest step changes in line scan
byte maxsteploc,minsteploc;  //tracks location of largest step change in linescan (pixel)
byte width;  //width of the filament in pixels
long res; // temp variable for calibration calculations
long width_um; // calibrated width in micrometers
long pwm;
int bandgap_measurement;

byte x0,x1,x2,x3,ct;
SQ7x8 a1,b1,c1,a2,b2,c2,m1,m2; //sub pixel quadratic interpolation variables in fixed point
SQ7x8 widthsubpixel,widthsubpixellp,test;

SQ7x8 width_calib, width_calib_fact;


//Function to treat fixed point as an int
int SQ7x8_as_int(SQ7x8 fp_number){
  int *int_num = (int *)(&fp_number);
  return(int_num[0]);
}

//Function to treat int as fixed point
SQ7x8 int_as_SQ7x8(int int_number){
  SQ7x8 *fp_num = (SQ7x8 *)(&int_number);
  return(fp_num[0]);
}

int low_pass_filter(int input, unsigned int parameter, long int *storage){

  long int temp1, temp2;

  temp1 = input;
  temp1 = temp1 <<  8;
  temp1 = temp1*parameter;
  temp2 = (*storage>>8 )* (long)(256-parameter) ;
  
  *storage = (long)temp1 + (long)temp2;

  return((int)(*storage >>16 ));
  
}


SQ7x8 low_pass_filter_fp(SQ7x8 input, unsigned int parameter, long int *storage){

int *int_num = (int *)(&input);
int res;
SQ7x8 *fp_num = (SQ7x8 *)(&res);
long int temp1, temp2;


  temp1 = int_num[0];
  temp1 = temp1 <<  8;
  temp1 = temp1*parameter;
  temp2 = (*storage>>8 )* (long)(256-parameter) ;
  
  *storage = (long)temp1 + (long)temp2;

  res = *storage >> 16 ;

  return(fp_num[0]);
  
}






void setup() {

  //Need to add code to delay turning on the 3v regulator if the button
  //is pressed at startup.  Allows ISP programming.
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(POW_EN_PIN, OUTPUT);
  pinMode(DISPLAY_LED_PIN, OUTPUT);

  digitalWrite(POW_EN_PIN, LOW); // turn off 3.3v power

  Serial.begin(115200);
  Serial.println("");
    
  timer = 0;
  button_press=0;
  while (millis()< 3000 || button_press){
    if (digitalRead(BUTTON_PIN)==0)
      button_press = 1;

    if (button_press){
      timer = timer +1;
      delay(4);
      digitalWrite(DISPLAY_LED_PIN, timer&16); //flash the LED
    }
  }

  timer = 0;
  digitalWrite(POW_EN_PIN, HIGH); // turn on 3.3v power
  

  iic_data.value.reserved = 0;
  

  //while(!Serial);  //wait for serial to start
  
  
  Serial.println("Filament Sensor Prototype MLX75306");
  

  
  

  pinMode(CS_PIN, OUTPUT);
  pinMode(FR_PIN,INPUT);

  pinMode(OUTPUT_PWM_PIN, OUTPUT);
  pinMode(LED_PWM_PIN, OUTPUT);
  
  digitalWrite(CS_PIN, HIGH); // disable Chip Select

  pinMode(LED_SW1_PIN, OUTPUT);
  digitalWrite(LED_SW1_PIN, LOW); //Low grounds LED cathode
  

  

// On Pro Micro the pins for I2C are:
// SDA Pin 2, (PD1)
// SCL Pin 3, (PD0)

  //config_pwm();
  config_pwm1();
  config_pwm4();

  
 

  ls_init();
  
  //delay at least 500 microseconds
  delay(1);  //make this more precise later

  //load values from eeprom (or default if never written or corrupted)
  meas_type = eeprom_meas_type; 
  //iic_data.value.meas_type = meas_type; 
  width_calib_fact = eeprom_width_calib_fact;
  
  
  //Initialize slave I2C Interface
  
  Wire.begin(0x59, HARD_WIRE_MODE);   //Use slave address not commonly used by other devices see:
  // https://learn.adafruit.com/i2c-addresses/the-list or
  // https://i2cdevices.org/addresses


  // register the handling routines for I2C
  Wire.onReceiveData(receiveData);
  Wire.onRequestData(sendData);
  Wire.onReceiveAdx(receiveAdx);
  Wire.onRequestDataNack(receiveComplete); //use this to detect the receive is complete

  lastMicros = micros()-LOOP_TIME; //start right away
  
}


//I2C Handler Routines needed for the HardWire library
// With these handlers, the MPU will act like a typical I2C chip interface
// Master can send an address in the range 0x$01-0x$FE
// Master can then read data from slave's registers from that address onwards
// Master write is also possible, but not needed in this application
// Note that there is no checking on valid addresses.

void receiveAdx(){ //  Called for new message
  first_byte = 1;  //set flag that first byte is received
  i2c_address = NULL_ADDRESS;
}
void receiveData(unsigned char data){
  if( first_byte){  //If first byte of message, treat as register address pointed used to send data from
    i2c_address = data;
    first_byte = 0;
  }else{  // Allow master to write into slave register space
    // Commented out, since we don't want master to write to slave register space
    //iic_data.I2CArray[i2c_address] = data;
    //i2c_address = i2c_address +1;
 
  }
}

unsigned char sendData(){
  unsigned char retval;
  retval = iic_data.I2CArray[i2c_address];
  i2c_address = i2c_address +1; 
  return(retval); 
}

void receiveComplete(unsigned char numbytes){
  i2c_address = NULL_ADDRESS;
}


// the loop routine runs over and over again forever:
void loop() {

  loop_time = micros()- lastMicros;
  if (loop_time >= LOOP_TIME){
    lastMicros = micros();
       
    timer = timer + 1;
  
    if(keypress==0 || keypress>250){
      if(meas_type==0 || (timer&64)) // in ratiometric mode or pulse light
        digitalWrite(DISPLAY_LED_PIN, HIGH);
    }
  
    //Set the duty cycle of the LED PWM (Timer 4) based on latest requested light strength

    TC4H = (lightstrength >> LED_SHIFT_FACTOR) >> 8;
    LED_PWM = (byte) (lightstrength >> LED_SHIFT_FACTOR) & 0xFF;
    
  
    // Start data collection from the image sensor
    
    //Initialize the data collection
  
    maxstep=0;
    minstep=0;
    maxsteploc=255;
    minsteploc=255;
    maxpixel=0;
  
  

    
    
    //clear the sub-pixel buffers
    x0=0;
    x1=0;
    x2=0;
    x3=0;
    ct=-2;  //index to count samples  need to load buffer for 2 steps to subtract x2-x1
  
  
    
  
    ls_start_integration(SHUTTER_DELAY_US);
    //ls_start_integration(lightstrength);
  
    
    while(digitalRead(FR_PIN) == 0);  //wait for Frame to go high
    ls_read(line_image);
  
    for(pct=0;pct<=141;pct++){
      //Output the image to serial as a greyscale ascii art for debugging
      //Serial.write(grey[line_image[pct]/28]);
      if (line_image[pct]>maxpixel)
        maxpixel = line_image[pct];
  
      // update the sliding buffers
      x3=x2;  
      x2=x1;
      x1=x0;
      x0=line_image[pct];
      ct=ct+1;
  
      //simple pixel edge detection algorithm
            
      //check if detecting an edge
      if(ct>1 && ct<140) //ensure buffer is full
      {
      if(x1<x2)  
        {
        if(minstep<x2-x1)
        
        {
          minstep=x2-x1;
          minsteploc=ct;
          c1=x1-x0;
          b1=x2-x1;
          a1=x3-x2;
          
        }
        }
      else
      {
        if(maxstep<x1-x2)
        {
          maxstep=x1-x2;
          maxsteploc=ct;
          c2=x1-x0;
          b2=x2-x1;
          a2=x3-x2;
          
        }
        
      }
      }
      lastpixel=line_image[pct];  
    
    }
  
  
    maxpixelfinal=maxpixel;
  
    if(minstep>16 && maxstep>16)  //check for significant threshold
    {
      width=maxsteploc-minsteploc;
      
    }
    else
      width=0;
    if(width>100)  //check for width overflow or out of range ( 20pixels per mm, 5mm)
      width=100;  
  
   
    
  
    //sub-pixel edge detection using interpolation
        
    m1=((a1-c1)/(a1+c1-(b1*2)))/2;
    m2=((a2-c2)/(a2+c2-(b2*2)))/2;
  
    if(width>MIN_PIXEL_LIMIT)   //check for a measurement > cutoff value  otherwise treat as noise and output a 0
      widthsubpixel=(width)+m2-m1; 
    else
      widthsubpixel=0;
  
    //Do low pass filtering here
    //widthsubpixellp = widthsubpixel;
    //test = test*(255./256.)+ widthsubpixel*(1./256.);
  
    //widthsubpixellp = int_as_SQ7x8(low_pass_filter(SQ7x8_as_int(widthsubpixel), 1, &width_filter_storage));
    widthsubpixellp = low_pass_filter_fp(widthsubpixel, WIDTH_FILTER_PARAM, &width_filter_storage);
  
    //width_calib = widthsubpixellp * width_calib_fact;
    res = (long)SQ7x8_as_int(widthsubpixellp) * (long)SQ7x8_as_int(width_calib_fact) >> (long)16;
    width_calib = int_as_SQ7x8(res) + widthsubpixellp;


//calculate width in micrometers for IIC
    //  width_micrometers = width_calib * 1mm/20pixels *1000
    //  (int) width_micrometers = width_calib * 1000/20 /256
    //  (int) width_micrometers = width_calib * 25/128
    width_um = SQ7x8_as_int(width_calib);
    width_um = width_um * 25 >> 7;
    
    



    
  
  //Measure bandgap reference to calibrate output in absolute mode
    //bandgap_measurement = bandgap();
    bandgap_measurement = low_pass_filter(bandgap(),BANDGAP_FILTER_PARAM,&bandgap_filter_storage);
  
   
    
  
    if (meas_type==0){
  
  
      // For 5.0v supply
      // 3mm is 60 pixels.  scale that to a 3.00 volt output.
      // 5.0/3.0 *60 = 100 pixels full scale
      // 100 * 256 = 25600 convert to int (from fixed point)
      // D to A has 1023 counts full scale
      // counts = 1023*pixels_int/25600 
      // counts = 0.0399609*pixels_int
      // work out a long fraction
      // 0.0399609 * 2^19 = 20951.04
      // counts = pixels_int * 20951.04  / 2^19

      // counts = 1000*pixels_int/25600
      // 1000/25600  = 20480/2^19
   
      pwm = SQ7x8_as_int(width_calib);
      
      pwm = (pwm * 20480) >> 19; // for 5.0 v supply

      if (pwm >= PWM_BIAS)
        pwm = pwm - PWM_BIAS;
      OUTPUT_PWM = pwm;
  
      
    }
    else
    {
        //how to adjust output to absolute volts 
        // For pro micro (32U4) 5.0v supply, 1.10v bandgap reference
        // Compensate the output for a lower or higher Vcc.
        // Factor is (5.0/Vcc)
        // in * 1023 /25600 *(5.00/Vcc) = pwm
        // Vcc = 1.10 * 1023 / bandgap_measure
        // in * 1023/25600 * 5.0/1.10/ 1023 * bandgap_measure = pwm
        // in * bandgap_measure * 4.54545454/25600 = pwm
        
  
        // 4.54545454/25600 = 93.09090909 / 2^19
        // in * bandgap_measure * 93 >> 19

        //Further calibration based on measuring the bandgap voltage of a particular IC
        //For different bandgap voltages use a different factor:
        //     V   Factor
        //   1.04  98
        //   1.05  98
        //   1.06  97
        //   1.07  96
        //   1.08  95
        //   1.09  94
        //   1.10  93
        //   1.11  92
        //   1.12  91
        //   1.13  91
        //   1.14  90
         
  
        pwm = SQ7x8_as_int(width_calib);        
        pwm = pwm * bandgap_measurement;        
        pwm = (pwm * 94) >> 19; //for 1.07v bandgap

        if (pwm >= PWM_BIAS)
          pwm = pwm - PWM_BIAS;
        
        OUTPUT_PWM = pwm;
    
    }


    
    
    
   
    //calibrate lightstrength
    if(maxpixel<250 && lightstrength<(LED_PERIOD_TICKS<<LED_SHIFT_FACTOR))
      lightstrength=lightstrength+1;
    else if(maxpixel>250 && lightstrength>0)
      lightstrength=lightstrength-1;
  
    //calibrate width
    if(digitalRead(BUTTON_PIN)==0){   //calibration button is pressed
      if(keypress<255)
        keypress=keypress+1;
      if(keypress==250){  //wait until key held for 2 sec
        if(widthsubpixellp > CALIB_STANDARD_THRESHOLD_B)
          width_calib_fact=CALIB_STANDARD_B/widthsubpixellp;
        else if(widthsubpixellp > CALIB_STANDARD_THRESHOLD_A)
          {
            //width_calib_fact=CALIB_STANDARD_A/widthsubpixellp;
              width_calib_fact = (CALIB_STANDARD_A - widthsubpixellp);
              res =  ((long)SQ7x8_as_int(width_calib_fact) << (long)16) / (long)SQ7x8_as_int(widthsubpixellp);
              width_calib_fact = int_as_SQ7x8(res);
          }
        else
          //width_calib_fact=1.0;
          width_calib_fact=0.0;
        eeprom_width_calib_fact = width_calib_fact;  //write to eeprom
  
        //Serial.print("Write to eeprom");
        //Serial.print(" width_calib_fact:");
        //Serial.print(static_cast<float>(width_calib_fact));
        //Serial.print(" measured:");
        //Serial.print(static_cast<float>(widthsubpixellp));
        //Serial.print(" expected:");
        //Serial.print(static_cast<float>(CALIB_STANDARD_A));
        //Serial.println();
       }   
    }else{
      if(keypress>20 && keypress<100){  //short press is released
        if(meas_type==0)
          meas_type=1;
        else
          meas_type=0;
        
        eeprom_meas_type = meas_type;  //write to eeprom
        //Serial.print("Write to eeprom");
        //Serial.print(" meas_type:");
        //Serial.print(meas_type);
        //Serial.println();
      }
      keypress=0;
    }  
  
    digitalWrite(DISPLAY_LED_PIN, LOW); //turn off display led

    //Output the measurements on the serial port
    Serial.print("$,");
    Serial.print(width_um);   
    Serial.print(",");
    Serial.print(lightstrength);
    
    Serial.println();
     
    


    

    // Update I2C register values to make available for master to read
    cli(); //do not allow interrupts, since it could be an IIC read
    iic_data.value.width_micrometers = (int)width_um;
    iic_data.value.lightstrength = (int)lightstrength;
    sei(); //resume interrupts
    
  
    

  }

  
}

byte ls_init(){
  byte sanity_byte, result1, result2;

  digitalWrite(CS_PIN, HIGH);
  SPI.begin ();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  sanity_byte = SPI.transfer(MLX75306_CR);
  result1 = SPI.transfer(0x00);
  result2 = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return(sanity_byte);  
}



byte ls_start_integration(unsigned int tm){
// tm is in microseconds
  byte sanity_byte, result1, result2;

  if (tm>6553)
    tm=6553;
    
  unsigned int T = tm * 10 + 4;
  
  digitalWrite(CS_PIN, LOW);
  sanity_byte = SPI.transfer(MLX75306_SI);
  result1 = SPI.transfer(T >> 8);
  result2 = SPI.transfer(T & 0xFF);
  digitalWrite(CS_PIN, HIGH);
  return(sanity_byte);  
}

byte ls_read(byte *data){

  byte sanity_byte, result1, result2;
  byte i, resultx;
  
  digitalWrite(CS_PIN, LOW);
  sanity_byte = SPI.transfer(MLX75306_RO8);
  result1 = SPI.transfer(2);
  result2 = SPI.transfer(143);

  for(i=10;i>0;i--){
    resultx = SPI.transfer(0x00);    
  }
  for(i=142;i>0;i--){
    data[i-1] = SPI.transfer(0x00);
  }

  for(i=4;i>0;i--){
    resultx = SPI.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
  return(sanity_byte);
}



void config_pwm1() {
//Config timer 1A (16 bit)


    TCCR1B = 0x00;                 // stop Timer1 clock for register updates

    // set the period and duty cycle
    ICR1 = DA_PERIOD_TICKS;           // PWM period
    OCR1A = DA_PERIOD_TICKS / 2;      // ON duration = drive pulse width = 50% duty cycle

    TCCR1A = _BV(COM1A1) |  _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | 0x01;  //turn on clock source with /1 prescaler to start timer.


}


void config_pwm4() {

  unsigned char sreg;
//Config timer 4B (fast pwm 8 bit)


    TCCR4B = 0x00;                 // stop Timer1 clock for register updates

    while (PLLCSR & _BV(PLOCK) == 0);  //Check that PLL is locked

    PLLFRQ = PLLFRQ | _BV(PLLTM0);  //set Timer 4 clock to use the faster PLL clock, divide by 1 (48MHz)

    
    // Set top of counter
    TC4H = LED_PERIOD_TICKS >> 8;
    OCR4C = (byte) LED_PERIOD_TICKS & 0xFF;
    
    // set the duty cycle
    TC4H = (LED_PERIOD_TICKS/2) >> 8;
    OCR4B = (byte)(LED_PERIOD_TICKS/2) & 0xFF;  //50%
     
    TCCR4A = _BV(COM4B0)| _BV(PWM4B);
    
    
    TC4H = 0;
    TCNT4 = 0; //reset counter
    TCCR4B =  _BV(CS40);  //turn on clock source with /1 prescaler to start timer.

}


//checks the level of Vcc and compares to threshold.
// see https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

word bandgap() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    word result = (high<<8) | low;

    return result; //bandgap measurement with VCC as a reference.  Bandgap is 1.1v

}
