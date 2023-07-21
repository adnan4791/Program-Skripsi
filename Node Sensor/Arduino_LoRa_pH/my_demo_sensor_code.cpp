#include "my_demo_sensor_code.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SimpleTimer.h>

SimpleTimer timer;

float calibration_value = 21.34 - 0.7;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;
const int ph_Pin = A0;
float Po = 0;
float PH_step;
int nilai_analog_PH;
double TeganganPh;

//untuk kalibrasi
float PH4 = 3.43;
float PH7 = 3.04;

float ph_act;

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NOMEMCLATURE, HERE TC WOULD MEAN TEMPERATURE IN CELCIUS FOR INSTANCE
// USE A MAXIMUM OF 3 CHARACTERS 
// 
// The node will be sending for instance \!TC/23.5 

char nomenclature_str[4]="PH";
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// ADD HERE SOME INITIALIZATION CODE
// HERE WE JUST DECLARE VALUE_PIN_READ AS INPUT PIN

void sensor_Init() {

  // for the temperature sensor
  pinMode(PIN_READ, INPUT);
//  pinMode(PIN_POWER, OUTPUT);
}
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE WAY YOU READ A VALUE FROM YOUR SPECIFIC SENSOR
// HERE IT IS AN EXAMPLE WITH THE LM35DZ SIMPLE ANALOG TEMPERATURE SENSOR

double sensor_getValue() {

//  //read the raw sensor value
//  int value = analogRead(PIN_READ);

  int nilai_analog_PH = analogRead(ph_Pin);
  TeganganPh = 5 / 1024.0 * nilai_analog_PH;

  PH_step = (PH4 - PH7) / 3;
  Po = 7.00 + ((PH7-TeganganPh) / PH_step);
 
//  ph_act = -5.70 * volt + calibration_value;

// Serial.println("pH Val: ");
// Serial.print(Po,2);  
    
  return Po;
}
///////////////////////////////////////////////////////////////////
