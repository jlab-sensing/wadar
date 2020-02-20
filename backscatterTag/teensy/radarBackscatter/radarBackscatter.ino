///GLOBALS///

//timer variables
elapsedMicros count; // frequency timing
elapsedMicros cycles; // toggle timing

const int period =  6250; // half period of tag in microseconds
//const int period =  10000; // half period of tag in microseconds
//int pn[] = {1,0,1,0,1,0,1,0};
int pn[] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
//int pn[] = {0,1,1,0,1,0,1,0,1,1,1,0,0,0,0,1,0,1,1,1,1,0,0,1,1,0,1,0,1,0,1,0};
const int tOn = (30000000); // tag on duration
const int tOff = (30000000); // tag off duration
const int tagPin = 3; // arduino pin supplying power to RF switch

//RF state variable - use for debugging
int RF = 0;
int i = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(tagPin, OUTPUT); //V_ctl
  //initialize RF state
  digitalWrite(tagPin, LOW);
}

void toggleRF() {
  // Toggle RF between On and Off at 80.0 Hz
  if (cycles >= period) {
    //digitalWrite(tagPin, (RF) ? HIGH : LOW);
    digitalWrite(tagPin, (pn[i]) ? HIGH : LOW);
    i = (i+1)%32;
    RF = !RF;
    //Serial.println(pn ? HIGH : LOW); //debugging
    cycles = 0; //restart fq timer after every half period
  }
}

void loop() {
//  if (count <= tOn) {
//    toggleRF();
//  } else if (count <= (tOn + tOff)) {
//    digitalWrite(tagPin, LOW); //turn tag off
//  } else {
//    count = 0; //restart toggle after one on/off cycle
//  }
  toggleRF();
}
