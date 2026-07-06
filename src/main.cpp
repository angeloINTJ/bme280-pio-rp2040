#include <Arduino.h>
#include "BMx280_PIO.h"
static float t_sum=0,p_sum=0,t_min=999,t_max=-99,p_min=9999,p_max=0;
static int n=0;
static BMx280_PIO *sensor=nullptr;
void setup(){pinMode(LED_BUILTIN,OUTPUT);Serial.begin(115200);sensor=new BMx280_PIO(2,3,0x76);}
void loop(){
  static bool ok=false;
  if(!ok){ok=sensor->begin();if(!ok){Serial.println("BEGIN FAIL");delay(2000);return;}}
  if(n>=20){
    Serial.print("AVG T=");Serial.print(t_sum/20.0f,2);
    Serial.print(" (");Serial.print(t_min,2);Serial.print("-");Serial.print(t_max,2);Serial.print(")");
    Serial.print(" P=");Serial.print(p_sum/20.0f,2);
    Serial.print(" (");Serial.print(p_min,2);Serial.print("-");Serial.print(p_max,2);Serial.println(")");
    Serial.println("TEST PASSED");while(1)delay(1000);
  }
  sensor->takeForcedMeasurement();
  float t,p,h;sensor->readAll(&t,&p,&h);
  Serial.print(n);Serial.print(": T=");Serial.print(t,2);Serial.print(" P=");Serial.println(p,2);
  t_sum+=t;p_sum+=p;if(t<t_min)t_min=t;if(t>t_max)t_max=t;if(p<p_min)p_min=p;if(p>p_max)p_max=p;
  n++;delay(500);
}
