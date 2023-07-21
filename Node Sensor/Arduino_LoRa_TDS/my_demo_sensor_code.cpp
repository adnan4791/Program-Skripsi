#include "my_demo_sensor_code.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "GravityTDS.h"

#define TdsSensorPin A1
GravityTDS gravityTds;

float temperature = 25,tdsValue = 0;




///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NOMEMCLATURE, HERE TC WOULD MEAN TEMPERATURE IN CELCIUS FOR INSTANCE
// USE A MAXIMUM OF 3 CHARACTERS 
// 
// The node will be sending for instance \!TC/23.5 

char nomenclature_str[4]="TDS";
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// ADD HERE SOME INITIALIZATION CODE
// HERE WE JUST DECLARE VALUE_PIN_READ AS INPUT PIN

void sensor_Init() {

  // for the temperature sensor
  pinMode(PIN_READ, INPUT);
//  pinMode(PIN_POWER, OUTPUT);
//setup tds
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE WAY YOU READ A VALUE FROM YOUR SPECIFIC SENSOR
// HERE IT IS AN EXAMPLE WITH THE LM35DZ SIMPLE ANALOG TEMPERATURE SENSOR

double sensor_getValue() {

//  //read the raw sensor value
//  int value = analogRead(PIN_READ);

  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  tdsValue = gravityTds.getTdsValue();  // then get the value
//  Serial.print(tdsValue,0);
    
  return tdsValue;
}
///////////////////////////////////////////////////////////////////
