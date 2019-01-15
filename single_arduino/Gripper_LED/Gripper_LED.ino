/*

 * rosserial Servo Control Example
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
/*
 * 120 degrees drops the first kit
 * 80 degrees is drop the second
*/

#include <FastLED.h>

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>




// Number of LED in each arm
#define NUM_LEDS 1

// Pins where LEDs are connected
#define DATA_PIN 2 

// Kill Switch Pin
const int killPin  = 8;


CRGB leds[NUM_LEDS];

ros::NodeHandle  nh;
Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  servo.attach(13); //attach it to pin 9

  // LED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Serial for kill switch
  Serial.begin(57600);
  pinMode(killPin, OUTPUT);
  
}


byte byteRead;


void loop(){
  Serial.println("MAIN LOOP");
  while (Serial.available() > 0)
  {
    Serial.println("INSIDE WHILE LOOP");
    byteRead = Serial.read();
    while (byteRead != 'K')
    {
        nh.spinOnce();
        delay(1);

        // Make LED orange for 0.3 sec
        leds[0] = CRGB::OrangeRed;
        FastLED.show();
        delay(300);

        // Turn LED off for 0.2 sec
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(200);
        byteRead = Serial.read();
        Serial.println("READING");

    }
    if (byteRead == 'K')
    {
      Serial.println(byteRead);
      Serial.println("Drone is turning off");
      digitalWrite(killPin, HIGH);
      leds[0] = CRGB::FireBrick;
      FastLED.show();
      
    }
  }
  




}
