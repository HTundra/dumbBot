#include <Servo.h>
 //FloorCleaner 2.2
//Kent Bacatan 2021
typedef struct {
  int lDist;
  int rDist;
}posData;
Servo myservo; // create servo object to control a servo

const int trig = 6; //trig pin in ultrasonic sensor
const int echo = 5; //echo pin in ultrasonic sensor
const int redL = 10; //input A of motor
const int redR = 11; //input C of motor
const int blackL = 13; //input B of motor
const int blackR = 12; //input D of motor. lui == backwards(?)
const int delayTime = 1250;

int fCount = 0; //counts the number of cycles that have been completed
int bWard = 0;
int serialBaudRate = 9600;
//int dongcoservo = 9; commented because declared but never referenced

int measureDist(); //measures distance using sensor
posData lookAround();
posData reverseAndScan();
void moveForward();
void rotateRight(int time); //move right
void rotateLeft(int time); //move left
void moveBackandForth(int lDist, int rDist); //move back and forth
void resetServo(); //reset servo
void moveBackward();
void stopMovement();
void startupSubroutine();
int servoRight(); //moves servo right
int servoLeft(); //moves servo left

void setup() {
  Serial.begin(serialBaudRate);
  myservo.attach(9);
  //sensor IO declaration
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  //motor output declaration
  pinMode(redL, OUTPUT);
  pinMode(redR, OUTPUT);
  pinMode(blackR, OUTPUT);
  pinMode(blackL, OUTPUT);
  //sets motor output pins to LOW
  digitalWrite(redL, LOW);
  digitalWrite(redR, LOW);
  digitalWrite(blackR, LOW);
  digitalWrite(blackR, LOW);
  //turns servo 90 degrees
  startupSubroutine();
  myservo.write(90);
  Serial.print("I'm awake!");
  delay(500);
}

void loop() {
  int max = 10;
  int longRange = 15;
  int dist = 0;
  posData biDist;
  dist = measureDist();
  moveForward();
  dist = measureDist();
  fCount++;
  if (fCount > 150 || dist < max) {
    stopMovement(); //Stops movement
    biDist = lookAround();
    fCount = 0;
    bWard++;
    if (bWard > 3) {
      bWard = 0;
      biDist = reverseAndScan();
      moveBackandForth(100, 1);
    } else if (((biDist.rDist) < max && (biDist.lDist) < max) || biDist.lDist == biDist.rDist || biDist.lDist == biDist.rDist + 1 || biDist.lDist == biDist.rDist - 1) {
      bWard = 0;
      biDist = reverseAndScan();
      if (biDist.lDist > longRange & biDist.rDist > longRange) {
        moveBackandForth(100, 1);
      } else {
        moveBackandForth(biDist.lDist, biDist.rDist);
      }
    } else if ((biDist.lDist) < max || (biDist.rDist) < max) { //if distance of left and right are less than 30
      moveBackandForth(biDist.lDist, biDist.rDist); //move back and forth
    }

    //   else {
    //      if (rDist >= lDist) { //if distance to the right is >= left
    //        rotateRight(delayTime); //move to the right
    //      }
    //      if (rDist < lDist) { //if distance to the left is < right
    //        rotateLeft(delayTime); //move to the left
    //      }
    //    }
  }
}

void resetServo() {
  myservo.write(90);
}

void startupSubroutine() {
  posData startupDist;
  moveBackward();
  delay(1000);
  stopMovement();
  startupDist = lookAround();
  moveBackandForth(startupDist.lDist, startupDist.rDist);
}

posData lookAround() {
  resetServo(); //resets the servo
  posData A;
  A.lDist = servoLeft(); //rotates the servo to the left
  A.rDist = servoRight(); //rotates the servo to the right
  return A;
}

posData reverseAndScan() {
  posData A;
  moveBackward();
  delay(1000);
  stopMovement();
  A = lookAround();
  return A;
}

void moveBackandForth(int lDist, int rDist) //I redid this function lmao maybe it's wrong.
{
  Serial.println("moveBackandForth");
  int i;
  resetServo();
  stopMovement();
  if (lDist > rDist) {
    rotateLeft(delayTime);
  } else if (rDist > lDist) {
    rotateRight(delayTime);
  }
  stopMovement();
}

int measureDist() {
  Serial.println("measureDist");
  int ret;
  unsigned long time;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  time = pulseIn(echo, HIGH); // Measures the time it takes from the sensor to return a HIGH value
  // ret = time / 2 / 29.412;
  ret = microsecondsToInches(time);
  Serial.println(ret);
  return ret;
}

int servoLeft() {
  Serial.println("servoLeft");
  int ret, var1, var2;
  myservo.write(180);
  delay(1000);
  var1 = measureDist();
  delay(500);
  myservo.write(135);
  delay(1000);
  var2 = measureDist();
  delay(500);
  myservo.write(90);
  var1 > var2 ? ret = var2 : ret = var1;
  return ret;
}

int servoRight() {
  Serial.println("servoRight");
  int ret, var1, var2;
  myservo.write(0);
  delay(1000);
  var1 = measureDist();
  delay(500);
  myservo.write(45);
  delay(1000);
  var2 = measureDist();
  delay(500);
  myservo.write(90);
  var1 > var2 ? ret = var2 : ret = var1;
  return ret;
}

void moveBackward() {
  Serial.println("moveBackward");
  digitalWrite(redL, HIGH);
  digitalWrite(redR, HIGH);
}

void moveForward() {
  Serial.println("moveForward");
  digitalWrite(blackR, HIGH);
  digitalWrite(blackL, HIGH);
}

void stopMovement() {
  Serial.println("stopMovement");
  digitalWrite(redL, LOW);
  digitalWrite(blackL, LOW);
  digitalWrite(redR, LOW);
  digitalWrite(blackR, LOW);
}

void rotateLeft(int time) {
  Serial.println("rotateLeft");
  stopMovement();
  digitalWrite(blackL, LOW);
  digitalWrite(blackR, LOW);
  digitalWrite(redL, HIGH);
  digitalWrite(redR, LOW);
  delay(time);
  digitalWrite(blackL, LOW);
  digitalWrite(blackR, HIGH);
  digitalWrite(redL, LOW);
  digitalWrite(redR, LOW);
  delay(time);
}

void rotateRight(int time) {
  Serial.println("rotateRight");
  stopMovement();
  digitalWrite(blackL, LOW);
  digitalWrite(blackR, LOW);
  digitalWrite(redR, HIGH);
  digitalWrite(redL, LOW);
  delay(time);
  digitalWrite(blackL, HIGH);
  digitalWrite(blackR, LOW);
  digitalWrite(redR, LOW);
  digitalWrite(redL, LOW);
  delay(time);
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}