#include "SDISerial.h"
 
 
#define INPUT_SIZE 30
#define NUMSAMPLES 5
#define DATA_PIN 13
#define INVERTED 1
 
int sensorDelay = 1000;
char* samples; 
 
 
SDISerial sdi_serial_connection(DATA_PIN, INVERTED);

 
void setup() {
  sdi_serial_connection.begin();
  Serial.begin(1200); 
  delay(3000);
}
 
void loop() {
 
  uint8_t i;
 
  // take repeated samples
  for (i = 0; i < NUMSAMPLES; i++) {
    samples = get_measurement();
    while (strlen(samples) < 5) {
      samples = get_measurement();  
    }
    Serial.print("samples(EC/RD/ST): ");
    Serial.println(samples);
  }
}
 
char* get_measurement(){
    // function by Joran Beasley: https://github.com/joranbeasley/SDISerial/blob/master/examples/SDISerialExample/SDISerialExample.ino
    char* service_request = sdi_serial_connection.sdi_query("?M!", sensorDelay);
    //you can use the time returned above to wait for the service_request_complete
    char* service_request_complete = sdi_serial_connection.wait_for_response(sensorDelay);
    // 1 second potential wait, but response is returned as soon as it's available
    return sdi_serial_connection.sdi_query("?D0!", sensorDelay);
}
