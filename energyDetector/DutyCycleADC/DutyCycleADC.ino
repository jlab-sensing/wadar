//Libraries 
#include <ResponsiveAnalogRead.h>;
#include <ADC.h>;

//Global Timing 
elapsedMicros count;

//Reading pin 
const int ANALOG_PIN = 37;
ADC *adc = new ADC(); 
int value = 0;
void setup() {
  Serial.begin(9600);
}

void loop() {
  if(count % 1000000 == 0){
    count = 0;
    adc->startContinuous(ANALOG_PIN, ADC_1); 
    if(count > 1){
       adc->stopContinuous(ADC_1);
    }
   value = (uint16_t)adc->analogReadContinuous(ADC_1); 
   Serial.println(value*3.3/adc->getMaxValue(ADC_1), DEC);

  }
}
