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
#define USE_USBCON
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
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  // pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  servo.attach(13); //attach it to pin 9
  
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

void loop(){
  nh.spinOnce();
  delay(1);


  // Make LED orange for 0.2 sec
  leds[0] = CRGB::OrangeRed;
  FastLED.show();
  delay(300);

  // Turn LED off for 0.2 sec
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(200);

}
