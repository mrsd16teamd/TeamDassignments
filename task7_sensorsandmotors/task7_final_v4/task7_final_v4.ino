/* MRSD Project Course
   Task 7: Motors and Sensors Lab
   Amit Bansal, Aum Jadhav, Cyrus Liu, Kazuya Otani, Max Hu
*/


#include <Stepper.h>
#include <Servo.h>
#include <Encoder.h>



//#include <digitalWriteFast.h>

/*
   Initialize variables, pins
*/
String outputStr;
//***** Sensors *****
#define button 4
bool btn1;
bool last_btn1;


bool bounce = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 250; //[ms]
int lastRead = 0;
unsigned long lastReadTime = 0;
unsigned long lastReadDelay = 200;

#define potPin A5
int potPast;

#define irPin A3
int ir_reading[255]; 
int ir_arraySize = 21;

#define tempPin A2
const float alpha = 0.5; //for low pass filter
float tempPast = 20.0; //change this if temp stays at 0
int tempInit;

#define sonarPin A1
int sonarPast = 0;
int sonarDist;
int arraySize = 5;
int reading[255]; 



//***** Servo motor *****
#define servoPin 10
int pos = 0;
int temp = 0;
int servoPos;
Servo servo;  // create servo object to control a servo

//***** Stepper motor *****
#define stepPin 8
#define dirPin 9
#define stepper_mode 1
#define stepper_EN 12
// mode 0: continuous rotation in one direction, with speed controlled by analogread
// mode 1: position control, input same as mode 0
const int stepsPerRev = 800; // (360 degrees/1.8stepangle)*4 (why 4? not sure. microstepping?)
int stepCount  = 0;
Stepper stepper(stepsPerRev, stepPin, dirPin);

//***** DC motor *****
#define dcMotorEnable 7
#define dcMotorPin1 5
#define dcMotorPin2 6
#define dcEncoder0PinA 2
#define dcEncoder0PinB 3
int dcEncoder0Pos = 0;
int dcEncoder0PinALast = LOW;
int dcN = LOW;

Encoder dcEnc(dcEncoder0PinA, dcEncoder0PinB);

double err, prev_err, total_err;


long last, now, t_step;

const double dcP = 1.0;
const double dcI = 0.001;
const double dcD = 0.5;

const double total_err_max = 50.0;
const double total_err_min = -50.0;

double dcPast = 0;
double dcDegree;
double dcRevCount;



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
  pinMode(stepper_EN, OUTPUT);
//  attachInterrupt(digitalPinToInterrupt(button), buttonPress, RISING);
  Serial.begin(57600);
  servo.attach(servoPin);
  digitalWrite(stepper_EN, 1); //disable the stepper until needed
  potPast = analogRead(potPin);
  tempInit = analogRead(tempPin);
}

//***** Read from pot. ******
int potRead() {       
  int potValue = alpha * analogRead(potPin) + (1 - alpha) * potPast; // low pass filter
  potPast = potValue;
  float angle = (float)potValue / 1023 * 360; //[degrees]
  delay(20);

  outputStr += String(potValue);
//  Serial.print(potValue);
//  Serial.println();
//  Serial.print('\t');
  return potValue;
}

int irRead() {          //***** Read IR sensor *****
//  int irValue = analogRead(irPin); //[cm]
//  float irDist = (6762 / (irValue - 9)) - 4;
//  irDist = int(irDist);
  
  for(int i = 0; i < ir_arraySize; i++)
  {                                                    //array pointers go from 0 to 4

    ir_reading[i] = analogRead(irPin);
    
    if (ir_reading[i] < 70) {
      ir_reading[i] = 70;
    }

//    delay(20);  //wait between analog samples
   } 
    //  sort the values in the array by ascending order
    for (int i=0; i< ir_arraySize-1; i++){
      for (int j=i+1; j< ir_arraySize; j++){
        if (ir_reading[i] > ir_reading[j]){
          int w = ir_reading[i];
          ir_reading[i] = ir_reading[j];
          ir_reading[j] = w;
        }
      }
     }

  int midpoint = ir_arraySize/2;    //midpoint of the array is the medain value in a sorted array
  int irValue = ir_reading[midpoint];
  
  float irDist = (6762 / (irValue - 9)) - 4;
  irDist = int(irDist);
  


  delay(20);  //wait between analog samples
  
//  Serial.println(irValue);
  outputStr += String(irValue);
  return irDist;
}

//***** Read temperature *****
int tempRead() {         
  int tempValue = analogRead(tempPin);
  float voltage = tempValue * (5.0 / 1023.0);
  float tempRead = (voltage * 1000 - 500) / 10;
  //digital low pass filter
  float temp =  alpha * tempRead + (1 - alpha) * temp; //[celsius]
//  Serial.println(tempValue);
  outputStr += String(tempValue);
  temp = int(temp);
  return temp;
  
}

//***** Read ultrasonic rangefinder *****
int sonarRead() {     

  for(int i = 0; i < arraySize; i++)
  {                                                    //array pointers go from 0 to 4

    reading[i] = analogRead(sonarPin)/2;

    delay(50);  //wait between analog samples
   } 
    //  sort the values in the array by ascending order
    for (int i=0; i< arraySize-1; i++){
      for (int j=i+1; j< arraySize; j++){
        if (reading[i] > reading[j]){
          int w = reading[i];
          reading[i] = reading[j];
          reading[j] =w;
        }
      }
     }


   // now show the median range   
   int midpoint = arraySize/2;    //midpoint of the array is the medain value in a sorted array
   sonarDist = reading[midpoint];
  int sonarValue = sonarDist * 2;
  outputStr += String(sonarValue);
//  Serial.println(sonarValue); 
  return sonarDist;
}

//***** Move servo *****
void servoControl(int motorInput) {       
  digitalWrite(dcMotorEnable, LOW);
//  analogWrite(dcMotorPin1, 0);
//  analogWrite(dcMotorPin2, 0);

  servoPos = map(motorInput, 0, 1023, 5 , 175);
//  Serial.println(servoPos);
  servo.write(servoPos);
//  servo.writeMicroseconds(servoPos);
//  delay(150);
}

//***** Move stepper *****
void stepperControl(int motorInput) {
  digitalWrite(dcMotorEnable, LOW);
//  analogWrite(dcMotorPin1, 0);
//  analogWrite(dcMotorPin2, 0);
  
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
//    Serial.println(stepsDesired);
    if (abs(stepsDesired) > 5) {
    stepper.step(-stepsDesired);
    }
//    delay(50);
  }
}

void dcEncoderRead() {

    dcEncoder0Pos = dcEnc.read();
    dcDegree = abs((dcEncoder0Pos/2) % 360);
//    Serial.print(dcEncoder0Pos);
//    Serial.print('\t');
//  Serial.print("dcDegree ");
//  Serial.print(dcDegree);
//  Serial.print('\t');

}

//***** DC Motor Position Control *****//
void dcPosControl(int motorInput) {
  digitalWrite(dcMotorEnable, HIGH);
  dcEncoderRead();
  
  int dcPosDes = map(motorInput, 0, 1023, 2, 358);
//  Serial.print("dcPositionDesired ");
//  Serial.print(dcPosDes);
//  Serial.print('\t');
  
  now = millis();
  t_step = now - last;
  prev_err = err;
  total_err += err;
  if (total_err > total_err_max) {
    total_err = total_err_max;
  }
  else if (total_err < total_err_min) {
    total_err = total_err_min;
  }
  err = dcPosDes - dcDegree;
//  double dcSpeed = err*dcP + err*t_step*dcI + (err/t_step)*dcD;
  double dcSpeed = err*dcP + total_err*t_step*dcI + (err-prev_err/t_step)*dcD;

//  Serial.println(total_err);

  
  dcSpeed = int(dcSpeed);
  
  if ((dcSpeed > -10) && (dcSpeed < 10)) {
    dcSpeed = 0;
  }  
  else if ((dcSpeed < 0) && (dcSpeed > -25)) {
    dcSpeed = -25;
  }
  else if ((dcSpeed > 0) && (dcSpeed < 25)) {
    dcSpeed = 25;
  }
  else if (dcSpeed > 127) {
    dcSpeed = 127;
  }
  else if (dcSpeed < -127) {
    dcSpeed = -127;
  }
//  Serial.print("dcSpeed: ");
//  Serial.println(dcSpeed); //only starts moving <102 and >152
  
  analogWrite(dcMotorPin1, 127 + dcSpeed);
  analogWrite(dcMotorPin2, 127 - dcSpeed);
  dcPast = dcDegree;
//  delay(10);
  last = millis();
//  Serial.print("t_step: ");
//  Serial.println(t_step);
}

//***** DC Motor Velocity Control *****//
void dcVelControl(int motorInput) {
  digitalWrite(dcMotorEnable, HIGH);
  dcEncoderRead();
  
  int dcSpeed = map(motorInput, 0, 1023, 0, 255);
  dcSpeed = int(dcSpeed);
  if ((dcSpeed < 132) && (dcSpeed > 122)) {
    dcSpeed = 127;
  }
  else if ((dcSpeed < 127) && (dcSpeed > 102)) {
    dcSpeed = 102;
  }
  else if ((dcSpeed > 127) && (dcSpeed < 152)) {
    dcSpeed = 152;
  }

  analogWrite(dcMotorPin1, dcSpeed);
  analogWrite(dcMotorPin2, 255 - dcSpeed);
//  Serial.println(dcSpeed);
}

void loop() {

  outputStr += String(sensorUsing);
  outputStr += " ";
  outputStr += String(motorUsing);
  outputStr += " ";
  
  btn1 = digitalRead(button);
  if (btn1 != last_btn1) {
    if (btn1) {
      motorUsing = ((motorUsing + 1) % numMotors) % 32767;
//      Serial.print("State change to ");
//      Serial.println(motorUsing);
    }
    delay(40);

    last_btn1 = btn1;
  }
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
      if (motorInput < 10) {
        motorInput = 10;
      }

      else if (motorInput >80) {
        motorInput = 80;
      }
      
      motorInput = map(motorInput,10,80,0,1023);
      break;
    
    case 2:
      motorInput = sonarRead();
      if (motorInput > 40){
        motorInput = 5;
        
      }
      else if (motorInput < 5) {
        motorInput = 5;
        
      }

      motorInput = map(motorInput,5,40,10,923);
      break;
    
    case 3:
      motorInput = int(tempRead());
      motorInput = map(motorInput, 0, 300, 0, 1023);
      break;
    default:
      motorInput = potRead();
      break;
  }

//  motorInput = alpha * motorInput + (1 - alpha) * motorInputPast;
//  motorInputPast = motorInput;

//  Serial.println(motorInput);

  switch(motorUsing) {
    
    case 0:
      digitalWrite(stepper_EN, 1);
      servo.attach(servoPin);
      servoControl(motorInput);
      break;
    case 1:
      servo.detach();
      digitalWrite(stepper_EN, 0); //Enable stepper
      stepperControl(motorInput);
      break;
    case 2:
      servo.detach();
      digitalWrite(stepper_EN, 1);
      dcPosControl(motorInput);
      break;
    case 3:
      servo.detach();
      digitalWrite(stepper_EN, 1);
      dcVelControl(motorInput);
      break;
    default:
      Serial.println("Invalid input!");
  }
  
  Serial.println(outputStr);
  outputStr = "";
  delay(20);
}

