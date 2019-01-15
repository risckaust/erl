/*
 * LED amber flashing + servo node
 * RISC lab 2019
 * 
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

CRGB leds[NUM_LEDS];

ros::NodeHandle  nh;
Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  // ROS servo node
  nh.initNode();
  nh.subscribe(sub);
  servo.attach(13); //attach servo to pin 13

  // LED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  
}

void loop(){

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
}
