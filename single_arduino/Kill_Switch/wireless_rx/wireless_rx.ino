const int killPin = 4;  // The pin that will deactivate relay

void setup() {
  Serial.begin(57600);
  pinMode(killPin, OUTPUT);
}

byte byteRead;

void loop() { 
  while (Serial.available() > 0)
  {
    byteRead = Serial.read();
    while (byteRead != 'K')
    {
        byteRead = Serial.read();
    }
    if (byteRead == 'K')
    {
      Serial.println(byteRead);
      Serial.println("Drone is turning off");
      digitalWrite(killPin, HIGH);
    }
    }
}
