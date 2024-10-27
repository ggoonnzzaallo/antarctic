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
  FRHip.attach(6); //Front Right Hip
  FRKnee.attach(5); //Front Right Knee

  FLShoulder.attach(4); //Front Left Shoulder
  FLHip.attach(3); //Front Left Hip
  FLKnee.attach(2); //Front Left Knee



//Initialized legs to standing
RRHip.write(5);
RRKnee.write(155);
// //Higher is CCW
// //Lower is CW


}

void loop() {
  // No additional logic in loop
}
