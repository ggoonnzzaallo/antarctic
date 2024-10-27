#include <Servo.h>

Servo FLKnee;  // Create servo objects
Servo FLHip;
Servo FLShoulder;
Servo FRKnee;
Servo FRHip;
Servo FRShoulder;
Servo RLKnee;
Servo RLHip;
Servo RLShoulder;
Servo RRKnee;
Servo RRHip;
Servo RRShoulder;

bool hasRun = false;  // Track if we've executed once

void setup() {

  delay(1000);

//Initilized shoulders to 90 degrees (default position)
  RRShoulder.write(90);
  RLShoulder.write(90);
  FRShoulder.write(90);
  FLShoulder.write(90);

  RRShoulder.attach(A1);  //Rear Right Shoulder
  RRHip.attach(12); //Rear Right Hip
  RRKnee.attach(11); //Rear Right Knee

  RLShoulder.attach(10); //Rear Left Shoulder
  RLHip.attach(9); //Rear Left Hip
  RLKnee.attach(8); //Rear Left Knee

  FRShoulder.attach(7); //Front Right Shoulder
  FRHip.attach(5); //Front Right Hip
  FRKnee.attach(6); //Front Right Knee

  FLShoulder.attach(4); //Front Left Shoulder
  FLHip.attach(2); //Front Left Hip
  FLKnee.attach(3); //Front Left Knee


//Hard coded stand

  RRHip.write(70);
  RRKnee.write(130);

  FRHip.write(70);
  FRKnee.write(130);

  RLHip.write(110);
  RLKnee.write(50);

  FLHip.write(110);
  FLKnee.write(50);

}

void loop() {
}
