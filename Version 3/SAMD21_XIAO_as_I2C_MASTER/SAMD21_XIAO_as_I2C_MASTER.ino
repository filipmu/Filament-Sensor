//Example code to communicate with the filament sensor using I2C

// On XIAO board the pins are:
// SDA Pin 4 PA8
// SCL Pin 5 PA9



#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)


    
}

byte dl, dh = 0;
int width_micrometers;
int light_intensity;
byte addr = 0;

void loop() {

  delay(10);

  
  Wire.beginTransmission(0x59); // transmit to device 0x59
  //Serial.print("Addr:");
  //Serial.print(addr);
  
  Wire.write(addr);        // send address byte
  Wire.endTransmission();    // stop transmitting
  delay(10);

  
  Wire.requestFrom(0x59, 5, true);    // request 5 bytes from slave device 0x59

  dl = Wire.read(); //dummy thats always 0 for some reason
  Serial.print(" Micrometers:");
  dl = Wire.read();
  dh = Wire.read();  
  width_micrometers = dl + (dh << 8);
  Serial.print(width_micrometers);         // print the character
  Serial.print(" Intensity:");
  dl = Wire.read();
  dh = Wire.read();
  Serial.print(",");
  light_intensity = dl + (dh << 8);
  Serial.print(light_intensity);         // print the character
  
  

  Serial.println(".");
  
  delay(10);
  
}
