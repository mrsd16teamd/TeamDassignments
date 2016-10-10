/* MRSD Project Course
   Task 7: Motors and Sensors Lab
   Amit Bansal, Aum Jadhav, Cyrus Liu, Kazuya Otani, Max Hu
*/


#include <Stepper.h>
#include <Servo.h>


/*
   Initialize variables, pins
*/

//***** Sensors *****
#define button 2
bool bounce = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 250; //[ms]
int lastRead = 0;
unsigned long lastReadTime = 0;
unsigned long lastReadDelay = 200;

#define potPin A5
int potPast;

#define irPin A4

#define tempPin A2
const float alpha = 0.5; //for low pass filter
float tempPast = 20.0; //change this if temp stays at 0
int tempInit;

#define sonarPin A1
int sonarPast = 0;
int sonarDist;

//***** Servo motor *****
#define servoPin 10
int pos = 0;
int temp = 0;
Servo servo;  // create servo object to control a servo

//***** Stepper motor *****
#define stepPin 8
#define dirPin 9
#define stepper_mode 1
// mode 0: continuous rotation in one direction, with speed controlled by analogread
// mode 1: position control, input same as mode 0
const int stepsPerRev = 800; // (360 degrees/1.8stepangle)*4 (why 4? not sure. microstepping?)
int stepCount  = 0;
Stepper stepper(stepsPerRev, stepPin, dirPin);

//***** DC motor *****
#define dcMotorEnable 7
#define dcMotorPin1 5
#define dcMotorPin2 6
#define dcEncoder0PinA 3
#define dcEncoder0PinB 4
int dcEncoder0Pos = 0;
int dcEncoder0PinALast = LOW;
int dcN = LOW;
const int dcP = 1.0;
const int dcD = 0.1;
int dcPast = 0;
int dcDegree;
int dcRevCount;



//***** Button and state parameters
// Button press changes motor being controlled
int motorInput = 0;
int motorInputPast = 0;
const int numMotors = 4; //0: servo. 1: stepper. 2: DC pos 3: DC vel
const int numSensors = 4;

//STATE PARAMETERS
int motorUsing = 0; //0=servo, 1=stepper, 2=dcpos, 3=dcvel
int sensorUsing = 0; //0=pot, 1=IR, 2=sonar, 3=temp


void setup() {
  pinMode(dcMotorPin1, OUTPUT);
  pinMode(dcMotorPin2, OUTPUT);
  pinMode(dcMotorEnable, OUTPUT);
  pinMode (dcEncoder0PinA, INPUT);
  pinMode (dcEncoder0PinB, INPUT);
  pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), buttonPress, RISING);
  Serial.begin(9600);
  servo.attach(servoPin);
  potPast = analogRead(potPin);
  tempInit = analogRead(tempPin);
}


//***** Button press interrupt stuff
void buttonPress() {
  bounce = check_bounce(); //debouncing
  if (!bounce) {
    motorUsing = ((motorUsing + 1) % numMotors) % 32767;
    //Serial.print("State change to ");
    //Serial.println(motorUsing);
  }
}
bool check_bounce()  // For button
//Returns true if the button rising is a bounce, false if it was actually pressed.
{
  if ( (millis() - lastDebounceTime) > debounceDelay) {
    if ( (lastRead == 0) && digitalRead(button) == 1 ) { //only do this for rising
      lastDebounceTime = millis();
      return false;
    }
  }
  else {
    return true;
  }
}

//***** Read from pot. ******
int potRead() {       
  int potValue = alpha * analogRead(potPin) + (1 - alpha) * potPast; // low pass filter
  potPast = potValue;
  float angle = (float)potValue / 1023 * 360; //[degrees]
  return potValue;
}

int irRead() {          //***** Read IR sensor *****
  int irValue = analogRead(irPin); //[cm]
  float irDist = (6762 / (irValue - 9)) - 4;
  if (irDist > 80 || irDist < 10)
    irDist = -1;
  return irValue;
}

//***** Read temperature *****
int tempRead() {         
  int tempValue = analogRead(tempPin);
  float voltage = tempValue * (5.0 / 1023.0);
  float tempRead = (voltage * 1000 - 500) / 10;
  //digital low pass filter
  float temp =  alpha * tempRead + (1 - alpha) * temp; //[celsius]
  return tempRead;
}

//***** Read ultrasonic rangefinder *****
int sonarRead() {     
  int sonarValue = analogRead(sonarPin); //[inches]
  if (abs(sonarValue - sonarPast) < 50)
    sonarDist = (sonarValue) / 2;
  else 
    sonarDist = sonarDist;
  return sonarValue;
}

//***** Move servo *****
void servoControl(int motorInput) {       
  digitalWrite(dcMotorEnable, LOW);
  analogWrite(dcMotorPin1, 0);
  analogWrite(dcMotorPin2, 0);

  int servoPos = map(motorInput, 0, 1023, 0, 180);
  servo.write(servoPos);
  delay(150);
}

//***** Move stepper *****
void stepperControl(int motorInput) {
  digitalWrite(dcMotorEnable, LOW);
  analogWrite(dcMotorPin1, 0);
  analogWrite(dcMotorPin2, 0);

  // mode 0: continuous rotation
  if (stepper_mode == 0) {
    int motorSpeed = map(motorInput, 0, 1023, 0, 100);
    if (motorSpeed > 0) {
      stepper.setSpeed(motorSpeed);
      stepper.step(-stepsPerRev / 100);
    }
  }

  // mode 1: position control
  if (stepper_mode == 1) {
    int sensorDiff = motorInput;
    int posDesired = (int)((float)sensorDiff / 1023 * stepsPerRev);
    int stepsDesired = posDesired - stepCount;
    stepCount = stepCount + stepsDesired;
    stepper.setSpeed(30);
    stepper.step(-stepsDesired);
    delay(50);
  }
}

void dcEncoderRead() {
  dcN = digitalRead(dcEncoder0PinA);
  if ((dcEncoder0PinALast == LOW) && (dcN == HIGH)) {
    if (digitalRead(dcEncoder0PinB) == LOW) {
      dcEncoder0Pos--;
    } else {
      dcEncoder0Pos++;
    }
    dcRevCount = dcEncoder0Pos / 180;
    //dcDegree = abs((dcEncoder0Pos)%180);
    dcDegree = abs((dcEncoder0Pos * 2) % 360);
    //Serial.print(dcEncoder0Pos);
    //Serial.print(' ');
    //Serial.println(dcDegree);
  }
  dcEncoder0PinALast = dcN;
}

//***** DC Motor Position Control *****//
void dcPosControl(int motorInput) {
  digitalWrite(dcMotorEnable, HIGH);
  dcEncoderRead();
  
  int dcPosDes = map(motorInput, 0, 1023, 0, 180);

  int dcSpeed = ((dcPosDes - dcDegree) * dcP + (dcDegree - dcPast) * dcD) * 255 / 360;
  analogWrite(dcMotorPin1, 127 - dcSpeed);
  analogWrite(dcMotorPin2, 127 + dcSpeed);
  dcPast = dcDegree;
}

//***** DC Motor Velocity Control *****//
void dcVelControl(int motorInput) {
  digitalWrite(dcMotorEnable, HIGH);
  dcEncoderRead();
  
  int dcSpeed = map(motorInput, 0, 1023, 0, 255);
  analogWrite(dcMotorPin1, dcSpeed);
  analogWrite(dcMotorPin2, 255 - dcSpeed);
}

void loop() {
  /*
     Read Serial input, change modes
  */
  char command;
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  if (command == 'm') {
    motorUsing = Serial.parseInt();
  }
  if (command == 's') {
    sensorUsing =  Serial.parseInt();
  }

  /*
     Read from sensors
  */

  //***** Assign value that will be sent to motors 0~1023, according to mode. *****
  switch (sensorUsing) {
    case 0:
      motorInput = potRead();
      break;
    case 1:
      motorInput = irRead();
      break;
    case 2:
      motorInput = sonarRead();
      break;
    case 3:
      motorInput = tempRead();
      break;
    default:
      motorInput = potRead();
      break;
  }

  motorInput = alpha * motorInput + (1 - alpha) * motorInputPast;
  motorInputPast = motorInput;

  Serial.println(motorInput);

  switch(motorUsing) {
    case 0:
      servoControl(motorInput);
      break;
    case 1:
      stepperControl(motorInput);
      break;
    case 2:
      dcPosControl(motorInput);
      break;
    case 3:
      dcVelControl(motorInput);
      break;
    default:
    Serial.println("Invalid input!");
  }
  

}

