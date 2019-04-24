///GLOBALS///
//timer variable
elapsedMicros count;
//RF state variable
int RF = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT); //V_ctl
  //initialize RF state
  digitalWrite(3, LOW);
}

void loop() {
  // Toggle RF between On and Off at 93.75 Hz
  if (count >= 5000) {
    digitalWrite(3, (RF) ? HIGH : LOW);
    RF = !RF;
    //Serial.println(RF);
    count = 0;
  }
}
