

#include <ros.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>


// Servo
Servo servo;


// Ros communication
ros::NodeHandle  nh;



/////-----------------Pump Connections----------------------------//
int in1 = 8;
int in2 = 7;


//--------------------Activate Vaccuum----------------------------//
int vaccuum_on (){
  digitalWrite(in1, HIGH);  ///TURN ON
  digitalWrite(in2, LOW);
}

int vaccuum_off (){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}


void subscriberCallback(const std_msgs::Int16& gripper_state_msg) {
  // the message will range from 0 to 80.
  int input_pos = gripper_state_msg.data;
      servo.write(10);
      if (input_pos > 20){
        vaccuum_on();
        delay(50);  
      } else if (input_pos <= 20){
        servo.write(30);
        delay(3000);
        vaccuum_off();
        
        
        } 
  }

// Subscribe to robotiq state
ros::Subscriber<std_msgs::Int16> sub("/Robotiq_state_arduino", &subscriberCallback);


void setup() {
  // Ros communication
  nh.initNode();
  nh.subscribe(sub);
  // Servo attach
  servo.attach(9);
  // Setup vaccuum
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  
// Init
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

}



void loop() {
///////////////////////////////////NOTES/////////////////////////////////////////////////
// Gripper will close and servo will go to position where vaccum will suck the air to ///
// activate jamming. When the gripper opens, the vaccum will pump air to reset the    ///
// jammer for better performance.                                                     ///
////////////////////////////////END OF NOTES/////////////////////////////////////////////
nh.spinOnce();

}
