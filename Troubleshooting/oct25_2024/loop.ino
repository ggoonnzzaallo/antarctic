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


}

void loop() {
    if (!hasRun) {
        for (int hipAngle = 0; hipAngle <= 125; hipAngle += 5) {
            RRHip.write(hipAngle);
            int index = hipAngle / 5;
            
            int kneeMin[] = {40,40,40,20,20,20,40,60,60,70,80,80,90,90,100,110,110,120,125,135,140,145,145,160,165,165};
            int kneeMax[] = {140,155,150,165,165,165,165,165,165,165,160,160,165,165,160,160,160,160,160,160,160,160,160,165,165,165};
            
            for (int knee = kneeMin[index]; knee <= kneeMax[index]; knee++) {
                RRKnee.write(knee);
                delay(15);
            }
        }
        hasRun = true;  
    }
}

