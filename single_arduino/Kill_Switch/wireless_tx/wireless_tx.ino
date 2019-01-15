void setup() {
  Serial.begin(57600);
  pinMode(7, INPUT);
}

void loop() {
  if(digitalRead(7) == HIGH) // if button is pressed
  {
   Serial.print("K");
   delay(10);
  }
}
