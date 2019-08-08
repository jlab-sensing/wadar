///GLOBALS///

//timer variables
elapsedMicros count; // frequency timing
elapsedMicros cycles; // toggle timing

const int period =  6250; // half period of tag in microseconds
const int tOn = (30000000); // tag on duration
const int tOff = (30000000); // tag off duration
const int tagPin = 3; // arduino pin supplying power to RF switch

//RF state variable - use for debugging
int RF = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(tagPin, OUTPUT); //V_ctl
  //initialize RF state
  digitalWrite(tagPin, LOW);
}

void toggleRF() {
  // Toggle RF between On and Off at 80.0 Hz
  if (cycles >= period) {
    digitalWrite(tagPin, (RF) ? HIGH : LOW);
    RF = !RF;
    //    Serial.println(RF); //debugging
    cycles = 0; //restart fq timer after every half period
  }
}

void loop() {
  if (count <= tOn) {
    toggleRF();
  } else if (count <= (tOn + tOff)) {
    digitalWrite(tagPin, LOW); //turn tag off
  } else {
    count = 0; //restart toggle after one on/off cycle
  }
}
