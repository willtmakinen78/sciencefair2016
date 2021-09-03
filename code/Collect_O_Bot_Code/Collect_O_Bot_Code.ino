#include <Servo.h>                                 //Libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_MotorShield AFMSDC(0x60);          //MotorShield Objects
Adafruit_MotorShield AFMSStepper(0x61);

Adafruit_DCMotor *frontRight = AFMSDC.getMotor(1);                  //DC Motor Objects
Adafruit_DCMotor *frontLeft = AFMSDC.getMotor(2);
Adafruit_DCMotor *backLeft = AFMSDC.getMotor(3);
Adafruit_DCMotor *backRight = AFMSDC.getMotor(4);
Adafruit_StepperMotor *stepper = AFMSStepper.getStepper(64, 2);     //Stepper Motor Object

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);   //assign ID to Magnetometer

Servo clawServo;      //Servo Objects
Servo armServo;

boolean go = true;

char dataArray[10];          //Serial Communication Variables
char inChar;
int index;
int cogX = 1;
boolean started = false;
boolean ended = false;

int stepperCount = 0;             //Searching Variables
boolean objectFound = false;
int returnStepperCount = 0;
int centerCount;
int centerCounter = 0;

float heading;                      //Magnetometer Variables
float declinationAngle = 0.188;
float headingDegrees;

int calculationCounter = 0;
float total = 0;
float average = 0;

float objectHeading;      //Position Variables
float objectHeadingLow;
float objectHeadingHigh;
float turnHeading;
float turnHeadingLow;
float turnHeadingHigh;
float cameraHeading;

boolean cameraLow = true;       //Centering Variables
boolean cameraHigh = true;

const int laser = 6;        //Laser Variables
const int laserLDR = 0;
int laserState;

const int encoder = 7;        //Collection Encoder Variables
int encoderState = 0;
int lastEncoderState = 0;
int turnCounter = 0;
int collectCounter = 0;
int slowCollectCounter = 0;
int returnCounter = 0;
int returnTurnCounter = 0;

const int rightLedge = 2;           //Obstacle Avoidance Variables
const int leftLedge = 3;
const int rightLedgeReset = 4;
const int leftLedgeReset = 5;
const int rightBumper = 18;
const int leftBumper = 19;

int rightLedgeState;       //Obstacle Avoidance States
int leftLedgeState;
int rightBumperState;
int leftBumperState = 0;

boolean leftBoundary = false;     //Large-scale Navigation Variables
boolean rightBoundary = false;
int sideMovement;
boolean var = true;
boolean blocked = false;
int forwardCounter = 0;
int netForwardCounter = 0;

void setup() {
  AFMSDC.begin();           //Starting Libraries
  AFMSStepper.begin();
  Serial.begin(9600);

  Serial.println("Object Collection Robot");

  if (!mag.begin()) {                           //will trigger if there the sensor is not detected
    Serial.println("No HMC5883 Detected");
    while (1);
  }

  clawServo.attach(10);      //attach the servos to their pins
  armServo.attach(9);

  pinMode(laser, OUTPUT);     //declare the laser pin as an output
  pinMode(encoder, INPUT);      //declare the encoder pin as an input

  pinMode(rightLedge, INPUT);           //declare the obstacle avoidance pins as inputs/outputs
  pinMode(leftLedge, INPUT);
  pinMode(rightLedgeReset, OUTPUT);
  pinMode(leftLedgeReset, OUTPUT);
  pinMode(rightBumper, INPUT);
  pinMode(leftBumper, INPUT);

  frontLeft->setSpeed(255);                   //set the speed of the motors
  frontRight->setSpeed(255);
  backLeft->setSpeed(255);
  backRight->setSpeed(255);

  Serial.println("Initialized");
}
void loop() {
  armServo.write(177);      //set the servos to their correct positions
  clawServo.write(90);

  Serial.println("Clearing First Row");
  row();                                         //search and collect objects in the robot's first row

  while (leftBoundary == false) {                   //will exit when an obstacle on the left (relatively speaking) is met
    Serial.println("Turning Left");

    left90();                                        //turn 90 degrees to the left

    moveSectionSide();                               //(facing left) move on to the next section using the function that tracks sideways movement
    if (blocked == false) {                          //if no obstacle is encountered:
      Serial.println("Turning Left");

      left90();                                     //turn 90 degrees to the left

      row();                                        //search for and collect any objects in the robot's current row
    }                                               //the robot will continue to move left one row at a time until an obstacle is met
    else {                                          //if an obstacle is met:
      Serial.println("Left Boundary Hit");
      leftBoundary = true;                          //adjust the variable and exit the loop
    }
  }                                                 //perform one more "row search" before turning in the other direction
  Serial.println("Performing One More Row Search Before Turning Around");
  Serial.println("Turning Left");

  left90();                                       //turn 90 degrees left

  row();                                          //perform the final row search

  Serial.println("Turning Right and Moving on to Other Half of Room");

  right90();                                      //turn 90 degrees right
  //                                              //move sideways (relatively speaking) past the starting row of the robot or until an obstacle is encountered:
  Serial.println("Move Past the Robot's Initial Starting Point or Until an Obstacle is Encountered");
  forwardCounter = 0;                             //reset the variables
  lastEncoderState = 0;

  digitalWrite(rightLedgeReset, HIGH);            //un-reset the 555 timers attached to the ledge sensors
  digitalWrite(leftLedgeReset, HIGH);

  digitalWrite(laser, HIGH);                      //turn the laser on

  while (forwardCounter < sideMovement) {         //the forwardCounter must be greater than the counter that was incremented during its sideways movement
    encoderState = digitalRead(encoder);          //get an initial reading of the encoder
    if (encoderState != lastEncoderState) {       //if the encoder wheel has moved:
      forwardCounter++;                           //increment the counter
    }                                               //distance or an obstacle can stop the robot's rightward movement:
    rightLedgeState = digitalRead(rightLedge);    //update the right ledge sensor state
    if (rightLedgeState == HIGH) {                //if there is a drop:
      forwardCounter = sideMovement;              //changee the variable, ending the loop
    }
    leftLedgeState = digitalRead(leftLedge);      //update the state of the left ledge sensor
    if (leftLedgeState == HIGH) {                 //if there is a drop:
      forwardCounter = sideMovement;              //change the variable, ending the loop
    }
    rightBumperState = digitalRead(rightBumper);  //update the state of the right bumper
    if (rightBumperState == HIGH) {               //if an obstacle is met:
      forwardCounter = sideMovement;              //change the variable, ending the loop
    }
    leftBumperState = digitalRead(leftBumper);    //update the state of the left bumper
    if (leftBumperState == HIGH) {                //if an obstacle is met:
      forwardCounter = sideMovement;              //change the variable, ending the loop
    }
    laserState = analogRead(laserLDR);              //update the state of the LDR
    if (laserState < 800) {                         //if the beam is broken:
      forwardCounter = sideMovement;                //update the counter, ending the while loop
    }
    forward();                                    //move forward until the loop exits

    lastEncoderState = encoderState;              //update the state of the variable
  }
  backward();                                     //brake
  digitalWrite(rightLedgeReset, LOW);             //reset the 555 timers attached to the ledge sensors
  digitalWrite(leftLedgeReset, LOW);
  digitalWrite(laser, LOW);                       //turn off the laser
  delay(50);
  Stop();                                         //stop the robot
  delay(100);                                    //will exit when an obstacle on the right (relatively speaking) is met

  while (rightBoundary == false) {
    moveSection();                                  //move on to the next section
    if (blocked == false) {                         //if no  obstacle is met:
      Serial.println("Turn Right");

      right90();                                    //turn 90 degrees to the right

      row();                                        // search for and collect all the objects in the robot's current row
    }
    else {                                          //if an obstacle is met:
      Serial.println("Right Boundary Hit");
      rightBoundary = true;                         //change the variable and exit the loop
    }
    Serial.println("Turning Right");                     //turn 90 degrees right

    right90();
  }                                                 //each loop the robot will move forward, turn right, clear its row, and then turn right
  Serial.println("Perform a Final Row Search");     // clear the final row:
  blocked = false;
  row();                                            // search for and collect all the objects in the robot's current row

  Serial.println("Room Cleared");
}
void search() {                                  //Search for Objects:
  Serial.println("Scanning");

  stepper->setSpeed(100);                      //set the speed of the stepper
  while (stepperCount < 2050) {               //the stepper must make more than 2050 steps for the loop to exit
    stepper->step(1, FORWARD, SINGLE);        //during each cycle of while loop,
    stepperCount++;                           //step the steper one step and incement the step counter
    delay(40);                                //allow time for RoboRealm to catch up

    serialCommunication();                    //read incoming data
    if (cogX > 310 && cogX < 330) {           //if the object is centered, stop the motor:
      Serial.println("Object Found");

      objectFound = true;                     //found obje
      //stepper->step(20, BACKWARD, SINGLE);    //correct overshoot

      centerCount = stepperCount;// - 20;        //save the number of steps it took to find the object and subtract 20 to account for the correction
      getHeading();                           //get the heading of the object
      objectHeading = average;                //save the heading of the object
      delay(100);                             //allow time for the stepper to stop

      Serial.println("Re-Centering Camera");    //Re-Center the Camera:
      while (centerCounter < centerCount) {   //will exit when the number of return steps equals the number of steps it
        stepper->step(1, BACKWARD, SINGLE);   //took to find the object
        centerCounter++;                      //move the stepper one step each loop and increment the counter
      }
      stepper->release();
      stepperCount = 2050;                    //exit the while loop
    }
    else {
      objectFound = false;
    }
  }
  if (objectFound == false) {               //return the stepper to its original position if no object was found
    Serial.println("No Object Found");
    stepperReturn();
  }
  stepperCount = 0;                         //reset the variables for next time
  cogX = 1;
  centerCounter = 0;
  Serial.println("Done");
  delay(100);                               //give time for the stepper to stop
}
void collect() {
  Serial.println("Collecting Object");

  stepperCount = 0;                              //reset all the variables
  objectFound = false;
  cameraHigh = true;
  cameraLow = true;
  turnCounter = 0;
  collectCounter = 0;
  slowCollectCounter = 0;
  returnCounter = 0;
  returnTurnCounter = 0;

  stepper->release();                           //turn off the stepper to reduce overheating

  objectHeadingLow = objectHeading - .5;        //create the threshold values
  objectHeadingHigh = objectHeading + .5;
  delay(100);

  Serial.println("Facing Object");                //rotate to face the object:

  encoderState = digitalRead(encoder);          //get an initial reading of the encoder
  lastEncoderState = encoderState;              //"zero out" the previous reading

  getHeading();                                 //get an initial reading of the robot's heading
  cameraHeading = average;
  while (cameraLow || cameraHigh == true) {    //both variables must be false for the loop to exit
    getHeading();
    cameraHeading = average;                    //update the heading of the camera
    if (cameraHeading > objectHeadingLow) {     //if the current heading is within range, adjust the variable accordingly
      cameraLow = false;
    }
    else {                                      //otherwise don't
      cameraLow = true;
    }
    if (cameraHeading < objectHeadingHigh) {    //if the current heading is within range, adjust the variable accordingly
      cameraHigh = false;
    }
    else {                                      //otherwise don't
      cameraHigh = true;
    }                                            //record how long the turn was for turning back later:
    encoderState = digitalRead(encoder);        //get an initial reading of the encoder
    if (encoderState != lastEncoderState) {     //if the encoder wheel has moved:
      turnCounter++;                            //increment the counter
    }
    right();                                    //turn right

    lastEncoderState = encoderState;            //update the variable for the next time through the loop
  }
  left();
  delay(50);
  Stop();                                       //stop th robot
  Serial.println("Picking Up Object");            //pick up the object:

  digitalWrite(laser, HIGH);                    //stop the robot and turn on the laser pointer
  clawServo.write(15);                          //open the claws
  delay(1000);                                  //give the servo time to move

  encoderState = digitalRead(encoder);          //get an initial reading of the encoder
  lastEncoderState = encoderState;              //"zero out" the previous reading

  laserState = analogRead(laserLDR);            //get an initial reading of the LDR
  while (laserState > 800) {                    //the beam must be broken for the loop to exit
    laserState = analogRead(laserLDR);          //update the reading of the LDR each time through the loop

    encoderState = digitalRead(encoder);        //get an initial reading of the encoder
    if (encoderState != lastEncoderState) {     //if the encoder wheel has moved:
      collectCounter++;                         //increment the counter
    }
    forward();                                  //move forward

    lastEncoderState = encoderState;            //update the variable for the next time through the loop
  }
  backward();                                   //a sort of brake
  delay(50);

  encoderState = digitalRead(encoder);          //get an initial reading of the encoder
  lastEncoderState = encoderState;              //"zero out" the previous reading
  //                                              //slow down before picking the object up:
  while (slowCollectCounter < 10) {             //the must be more than 10 changes on the encoder for the loop to exit
    encoderState = digitalRead(encoder);        //update the state of the encoder
    if (encoderState != lastEncoderState) {     //if there has been a change:
      slowCollectCounter++;                     //increment the secondary counter
    }
    forward();                                  //move forward

    lastEncoderState = encoderState;            //update the variable for the next time through the loop
  }
  Stop();                                       //stop the robot
  delay(100);                                   //give it time to stop
  pickUp();                                     //pickup the object

  Serial.println("Returning to starting position");

  encoderState = digitalRead(encoder);          //get an initial reading of the encoder
  lastEncoderState = encoderState;              //"zero out" the previous reading

  while (returnCounter < collectCounter + slowCollectCounter) {     //return to the starting position:
    encoderState = digitalRead(encoder);        //update the state of the encoder
    if (encoderState != lastEncoderState) {     //if there has been a change:
      returnCounter++;                          //increment the secondary counter
    }
    backward();                                 //move backward

    lastEncoderState = encoderState;            //update the variable for the next time through the loop
  }
  Stop();                                       //stop the robot
  delay(100);                                   //allow time for the robot to stop
  Serial.println("Facing Original Direction");    //rotate to face the original direction:
  while (returnTurnCounter < turnCounter) {     //the return turn number must be greater than the original turn number
    encoderState = digitalRead(encoder);        //update the state of the encoder
    if (encoderState != lastEncoderState) {     //if there has been a change:
      returnTurnCounter++;                      //increment the counter
    }
    left();                                      //turn left

    lastEncoderState = encoderState;             //update the variable for the next time through the loop
  }
  Stop();                                        //stop the robot in its starting position

  digitalWrite(laser, LOW);                     //turn the laser off as well
}
void row() {                      //scans and picks up objects that are in the current row of the robot
  Serial.println("Clearing Current Row");
  blocked = false;                //reset the variables
  var = true;
  Serial.println("Moving Forward Until an Obstacle is Met");
  while (blocked == false) {      //move forward in sections until an obstacle is met (blocked == true):
    Serial.println("Clearing Current Section");
    while (var == true) {         //an obstacle must be hit for the bigger loop to exit, the variable must be false for this one to exit
      search();                   //search for an object and re-center the camera if one is found
      if (objectFound == true) {  //if an object is found:
        collect();                //collect it
      }
      else {                       //otherwise:
        var = false;               //make the variable false and exit the small loop
      }
    }
    moveSection();                 //advance to the next section
  }                                //repeat until an obstacle is met
  Serial.println("Search for Objects One Last Time Before Turning Around");
  var = true;                        //after meeting an obstacle, search for objects one last time before turning around:
  while (var == true) {            //the variable must be false for the loop to exit
    search();                      //search for objects and re-center the camera
    if (objectFound == true) {     //if an object is cound:
      collect();                   //collect it and return the robot to its starting position
    }                              //this loop will repeat until no objects can be found
    else {                         //if no objects are found:
      var = false;                 //make the variable false and exit the loop
    }
  }
  Serial.println("Turning Around");                //turn 180 degrees to face the opposite direction:
  turn180();

  Serial.println("Move Forward to the Initial Starting Position");
  while (forwardCounter < netForwardCounter) {    // move forward past the initial starting point:
    encoderState = digitalRead(encoder);        //get an initial reading of the encoder
    if (encoderState != lastEncoderState) {     //if the encoder wheel has moved:
      forwardCounter++;                         //increment the counter
    }
    forward();                                  //move forward until the loop exits

    lastEncoderState = encoderState;            //update the state of the variable
  }
  backward();                                   //brake
  delay(50);
  Stop();                                       //stop the robot

  forwardCounter = 0;                           //reset the counter
  blocked = false;                              //reset the variables
  var = true;
  Serial.println("Move Backward Until an Obstacle is Met");
  while (blocked == false) {        //move backward until an obstacle is met:
    Serial.println("Clearing Current Section");

    while (var == true) {           //var must be true for the small loop to exit
      search();                     //search for objects and re-center the camera if any are found
      if (objectFound == true) {    //if an object is found:
        collect();                  //collect it
      }
      else {                        //otherwise:
        var = false;                //adjust var and exit the small loop
      }
    }
    moveSection();                  //advance to the next section
  }                                 //this will repeat until an obstacle is met
  var = true;                         //search for objects one last time before turning:
  Serial.println("Search for Objects One LastTime Before Moving on");
  while (var == true) {           //the variable must be false for the loop to exit
    search();                     //search for objects and re-center the camera if any are found
    if (objectFound == true) {    //if an object is cound:
      collect();                  //collect it and return the robot to its starting position
    }                             //this loop will repeat until no objects can be found
    else {                        //if no objects are found:
      var = false;                //make the variable false and exit the loop
    }
  }
  netForwardCounter = 0;          //reset the variable for next time
}
void moveSection() {                                  //Move the robot to the next section
  Serial.println("Moving on to Next Section");

  blocked = false;                                  //reset the variables
  forwardCounter = 0;

  digitalWrite(rightLedgeReset, HIGH);             //un-reset the 555 timers attached to the ledge sensors
  digitalWrite(leftLedgeReset, HIGH);

  digitalWrite(laser, HIGH);                       //turn on the laser

  while (forwardCounter < 280) {                    //the encoder has to change 300 times for the robot to move 4 feet
    encoderState = digitalRead(encoder);            //update the state of the encoder
    if (encoderState != lastEncoderState) {         //if it has changed:
      forwardCounter++;                             //increment the forwardCounter, used ofor stopping the robot at 4 feet
      netForwardCounter++;                          //incrementing the netForwardCounter is used for backtracking later, once an obstacle has been met
    }
    rightLedgeState = digitalRead(rightLedge);      //update the state of the right ledge sensor
    if (rightLedgeState == HIGH) {                  //if there is a drop:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    leftLedgeState = digitalRead(leftLedge);        //update the state of the left ledge sensor
    if (leftLedgeState == HIGH) {                   //if there is a drop:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    rightBumperState = digitalRead(rightBumper);    //update the state of the right bumper
    if (rightBumperState == HIGH) {                 //if it hits an obstacle:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    leftBumperState = digitalRead(leftBumper);      //update the state of the left bumper
    if (leftBumperState == HIGH) {                  //if it hits an obstacle:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    laserState = analogRead(laserLDR);              //update the state of the LDR
    if (laserState < 800) {                         //if the beam is broken:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    forward();                                       //move forward

    lastEncoderState = encoderState;                 //update the last known state for the next time through the loop
  }
  Serial.println("Object Encountered or Next Section Met");

  backward();                                        //brake
  digitalWrite(rightLedgeReset, LOW);                //reset the 555 timers attached to the ledge sensors
  digitalWrite(leftLedgeReset, LOW);
  digitalWrite(laser, LOW);                          //turn off the laser
  delay(50);
  Stop();                                             //stop the robot

  forwardCounter = 0;                                 //reset the counter for the next movement
}
void moveSectionSide() {                              //this function is identical to the previous one except:
  Serial.println("Moving on to Next Section");

  blocked = false;
  forwardCounter = 0;

  digitalWrite(rightLedgeReset, HIGH);             //un-reset the 555 timers attached to the ledge sensors
  digitalWrite(leftLedgeReset, HIGH);

  digitalWrite(laser, HIGH);                       //turn on the laser

  while (forwardCounter < 280) {
    encoderState = digitalRead(encoder);
    if (encoderState != lastEncoderState) {
      forwardCounter++;
      sideMovement++;                                 //each time it is incrementing sideMovement, used to track how far sideways
    }                                                 //(releatively speaking) the robot has moved
    digitalWrite(rightLedgeReset, HIGH);
    digitalWrite(leftLedgeReset, HIGH);
    rightLedgeState = digitalRead(rightLedge);
    if (rightLedgeState == HIGH) {
      forwardCounter = 300;
      blocked = true;
    }
    leftLedgeState = digitalRead(leftLedge);
    if (leftLedgeState == HIGH) {
      forwardCounter = 300;
      blocked = true;
    }
    rightBumperState = digitalRead(rightBumper);
    if (rightBumperState == HIGH) {
      forwardCounter = 300;
      blocked = true;
    }
    leftBumperState = digitalRead(leftBumper);
    if (leftBumperState == HIGH) {
      forwardCounter = 300;
      blocked = true;
    }
    laserState = analogRead(laserLDR);              //update the state of the LDR
    if (laserState < 800) {                         //if the beam is broken:
      forwardCounter = 300;                         //update the counter, ending the while loop
      blocked = true;                               //adjust the variable, used in other parts of the navigation
    }
    forward();

    lastEncoderState = encoderState;
  }
  Serial.println("Obstacle Encountered or Next Section Met");

  backward();
  digitalWrite(rightLedgeReset, LOW);
  digitalWrite(leftLedgeReset, LOW);
  digitalWrite(laser, LOW);
  delay(50);
  Stop();

  forwardCounter = 0;
}
void serialCommunication() {                //get the X-coordinate of the object
  while (Serial.available() > 0) {        //Read the serial port
    inChar = Serial.read();
    if (inChar == '<') {                  //look for "<" to start
      started = true;
      index = 0;
      dataArray[index] = '\0';            //Make the first character in the array null
    }
    else if (inChar == '>') {             //look for ">" to end
      ended = true;
    }
    else if (started == true) {           //if the array has started and the character is not "<" or ">"
      dataArray[index] = inChar;          //save it to the array and increment index so that the final
      index++;                            //character is null
      dataArray[index] = '\0';
    }
  }
  if (started && ended == true) {        //If the array is filled in:
    cogX = atoi(dataArray);              // Convert the string to an integer

    started = false;                     // Get ready for the next time
    ended = false;
    index = 0;
    dataArray[index] = '\0';
  }
}
void getHeading() {
  calculationCounter = 0;
  total = 0;
  average = 0;

  while (calculationCounter <= 10) {
    sensors_event_t event;                                    //create a new sensor event
    mag.getEvent(&event);

    heading = atan2(event.magnetic.z, event.magnetic.x);      //calculate heading when the magnetometer is level, then correct for signs of axis
    heading += declinationAngle;                              //compensate for declination angle
    if (heading < 0) {                                        //correct for reversal of signs
      heading += 2 * PI;
    }
    if (heading > 2 * PI) {                                   //correct wrap due to addition of declination angle
      heading -= 2 * PI;
    }
    headingDegrees = heading * 180 / M_PI;                    //convert from radians to degrees

    total += headingDegrees;
    calculationCounter++;
    delay(1);
  }
  average = total / 10;
}
void stepperReturn() {                           //return the stepper to its starting position:
  Serial.println("Returning Stepper to Original Position");
  stepper->setSpeed(100);                      //set the speed of the stepper
  while (returnStepperCount < 2050) {          //the stepper must make more than 2050 steps for the loop to exit
    stepper->step(1, BACKWARD, SINGLE);        //during each cycle of while loop,
    returnStepperCount++;                      //step the stepper one step and increment the step counter
  }
  stepper->release();                          //turn the stepper off
  returnStepperCount = 0;                      //reset the count for next time
  delay(100);                                  //allow time for the stepper to stop
}
void pickUp() {
  clawServo.write(180);
  delay(1000);
  armServo.write(10);
  delay(1000);
  clawServo.write(15);
  delay(1000);
  armServo.write(177);
  delay(1000);
  clawServo.write(90);
  delay(1000);
}
void right90() {                                  //turn 90 degrees to the right
  Serial.println("Turning 90 Degrees to the Right");

  cameraLow = true;                            //reset the variables
  cameraHigh = true;
  turnHeading = 0;
  turnHeadingLow = 0;
  turnHeadingHigh = 0;
  cameraHeading = 0;

  stepper->setSpeed(100);                       //set the speed of the stepper
  stepper->step(515, FORWARD, SINGLE);          //turn it 90 degrees right
  getHeading();
  turnHeading = average;                        //save the heading of the turn
  delay(100);
  stepper->step(515, BACKWARD, SINGLE);         //return the stepper to its original position

  turnHeadingLow = turnHeading - 1;            //create the threshold values
  turnHeadingHigh = turnHeading + 1;
  delay(100);

  getHeading();                                 //get an initial reading of the robot's heading
  cameraHeading = average;
  while (cameraLow || cameraHigh == true) {    //both variables must be false for the loop to exit
    getHeading();
    cameraHeading = average;                    //update the heading of the camera
    if (cameraHeading > turnHeadingLow) {       //if the current heading is within range, adjust the variable accordingly
      cameraLow = false;
    }
    else {                                      //otherwise don't
      cameraLow = true;
    }
    if (cameraHeading < turnHeadingHigh) {      //if the current heading is within range, adjust the variable accordingly
      cameraHigh = false;
    }
    else {                                      //otherwise don't
      cameraHigh = true;
    }
    right();                                    //turn right
  }
  Stop();                                       //stop the robot
}
void left90() {                                 //turn 90 degrees to the left
  Serial.println("Turning 90 Degrees to the Left");

  cameraLow = true;                            //reset the variables
  cameraHigh = true;
  turnHeading = 0;
  turnHeadingLow = 0;
  turnHeadingHigh = 0;
  cameraHeading = 0;

  stepper->setSpeed(100);                      //set the speed of the stepper
  stepper->step(515, BACKWARD, SINGLE);        //turn it 90 degrees to the left
  getHeading();
  turnHeading = average;                       //save the heading of the turn
  delay(100);
  stepper->step(515, FORWARD, SINGLE);         //return the stepper to its original position

  turnHeadingLow = turnHeading - 1;           //create the threshold values
  turnHeadingHigh = turnHeading + 1;
  delay(100);

  getHeading();                                 //get an initial reading of the robot's heading
  cameraHeading = average;
  while (cameraLow || cameraHigh == true) {     //both variables must be false for the loop to exit
    getHeading();
    cameraHeading = average;                    //update the heading of the camera
    if (cameraHeading > turnHeadingLow) {       //if the current heading is within range, adjust the variable accordingly
      cameraLow = false;
    }
    else {                                      //otherwise don't
      cameraLow = true;
    }
    if (cameraHeading < turnHeadingHigh) {      //if the current heading is within range, adjust the variable accordingly
      cameraHigh = false;
    }
    else {                                      //otherwise don't
      cameraHigh = true;
    }
    left();                                     //turn left
  }
  Stop();                                       //stop the robot
}
void turn180() {                                  //turn around 180 degrees
  Serial.println("Turning 180 Degrees");

  cameraLow = true;                            //reset the variables
  cameraHigh = true;
  turnHeading = 0;
  turnHeadingLow = 0;
  turnHeadingHigh = 0;
  cameraHeading = 0;

  stepper->setSpeed(100);                       //set the speed of the stepper
  stepper->step(1025, FORWARD, SINGLE);         //turn it 180 degrees right
  getHeading();
  turnHeading = average;                        //save the heading of the turn
  delay(100);
  stepper->step(1025, BACKWARD, SINGLE);        //return the stepper to its original position

  turnHeadingLow = turnHeading - 1;            //create the threshold values
  turnHeadingHigh = turnHeading + 1;
  delay(100);

  getHeading();                                 //get an initial reading of the robot's heading
  cameraHeading = average;

  while (cameraLow || cameraHigh == true) {     //both variables must be false for the loop to exit
    getHeading();
    cameraHeading = average;                    //update the heading of the camera
    if (cameraHeading > turnHeadingLow) {       //if the current heading is within range, adjust the variable accordingly
      cameraLow = false;
    }
    else {                                      //otherwise don't
      cameraLow = true;
    }
    if (cameraHeading < turnHeadingHigh) {      //if the current heading is within range, adjust the variable accordingly
      cameraHigh = false;
    }
    else {                                      //otherwise don't
      cameraHigh = true;
    }
    right();                                    //turn right
  }
  Stop();                                       //stop the robot
}
void forward() {                //move forward
  frontLeft->run(FORWARD);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(FORWARD);
}
void backward() {               //move backward
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(BACKWARD);
}
void Stop() {                   //stop
  frontLeft->run(RELEASE);
  frontRight->run(RELEASE);
  backLeft->run(RELEASE);
  backRight->run(RELEASE);
}
void right() {                  //turn right
  frontLeft->run(FORWARD);
  frontRight->run(BACKWARD);
  backLeft->run(FORWARD);
  backRight->run(BACKWARD);
}
void left() {                   //turn left
  frontLeft->run(BACKWARD);
  frontRight->run(FORWARD);
  backLeft->run(BACKWARD);
  backRight->run(FORWARD);
}
