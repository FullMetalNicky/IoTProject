#include "PERIPUMP.h"
#include "Servo.h"

#include <DFRobot_Geiger.h>
#if defined ESP32
#define detect_pin D3
#else
#define detect_pin 3
#endif

//PERIPUMP pump(5);
Servo s;


/*!
   @brief Constructor
   @param pin   External interrupt pin
*/
DFRobot_Geiger  geiger(detect_pin);

/*
void setupPump()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PERIPUMP_LIB_VERSION: ");
  Serial.println(PERIPUMP_LIB_VERSION);

  pump.begin();
  pump.stop();
}
*/

void setupServo()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println("PERIPUMP SERVO ONLY TEST");

  s.attach(5);   //  Connect to PIN 5


}

void fillContainer(int ptime)
{
  
  s.write(0);
  delay(ptime);
  s.write(90);
  delay(1000);


}

void drainContainer(int ptime)
{
  
  s.write(180);
  delay(ptime);
  s.write(90);
  delay(1000);


}

/*
void fillContainer(int ptime)
{

  int pos = analogRead(A0) - 512;   //  assumes  UNO 10 bits ADC
  pump.setPercentage(pos / 5.12);
  Serial.print(pos);
  Serial.print('\t');
  Serial.print(pump.getPercentage());
  Serial.println();

  pump.forward();
  delay(ptime);
  pump.stop();


}
*/



void setupGeiger()
{
  Serial.begin(115200);
  //Start counting, enable external interrupt
  geiger.start();
}

void setup()
{
  setupServo();
  //setupGeiger();

  

}
/*
void loop() {

  Serial.println("begin loop");
  fillContainer(50000);
  Serial.println("end of loop");
  drainContainer(10000);


}
*/


void loop(){ 
  //geiger.start();
  delay(3000);
  //geiger.pause();
  Serial.println(geiger.getCPM());
  Serial.println(geiger.getnSvh());
  Serial.println(geiger.getuSvh());
}
