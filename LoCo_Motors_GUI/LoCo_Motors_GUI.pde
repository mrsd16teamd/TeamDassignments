import processing.serial.*;
import controlP5.*;

// Serial Communication Variables
String portName = "";
String arduinoResponse;
Serial myPort;      // The serial port
int whichKey = -1;  // Variable to hold keystoke values
int inByte = -1;    // Incoming serial data
boolean firstContact = false;
int sensor_value = 0;

// GUI
ControlP5 cp5;
Textarea prompt;
Textarea serial_window;
String text = "";
String which_motor = "**Current Motor: Servo\n";
String which_sensor = "**Current Sensor: Pot\n";
int sensorUsing = 0;
String sensorPrompt = "***-:Angle: ";
String Unit = " degree";
float realSensorVal = 0.0;
String p = "";
String sout = "";
DropdownList portList;
Textfield sin;
Textlabel portLabel, sinLabel, soutLabel, promptLabel, senvalLabel;
Button stepper, dc_pos, dc_vel, servo;
Button sensor1, sensor2, sensor3, sensor0;

void setup() {
  size(1000, 600);
  smooth();
  printArray(Serial.list());
 
  cp5 = new ControlP5(this);
  
  servo = cp5.addButton("servo")
              .setPosition(700,70)
              .setSize(200,19);             
  stepper = cp5.addButton("stepper")
              .setPosition(700,90)
              .setSize(200,19);
  dc_pos = cp5.addButton("dc_pos")
              .setPosition(700,110)
              .setSize(200,19);
  dc_vel = cp5.addButton("dc_vel")
              .setPosition(700,130)
              .setSize(200,19);
              
  sensor0 = cp5.addButton("pot")
              .setPosition(700,190)
              .setSize(200,19);             
  sensor1 = cp5.addButton("IR")
              .setPosition(700,210)
              .setSize(200,19);
  sensor2 = cp5.addButton("sonar")
              .setPosition(700,230)
              .setSize(200,19);
  sensor3 = cp5.addButton("temp")
              .setPosition(700,250)
              .setSize(200,19); 

  sinLabel = cp5.addTextlabel("sinLabel")
                 .setText("Serial Input: ")
                 .setPosition(50,300)
                 .setColorValue(0xffffffff)
                 .setFont(createFont("orbitron",15));
                 
  sin = cp5.addTextfield("sin")
           .setPosition(55, 320)
           .setSize(200,40)
           .setFocus(true)
           .setColor(color(255,255,255))
           .setColorCursor(color(255))
           .setFont(createFont("courier", 15));
  Label label = sin.getCaptionLabel();
  label.setColor(192);
  
  portLabel = cp5.addTextlabel("portLabel")
                 .setText("Select Serial Port: ")
                 .setPosition(50,50)
                 .setColorValue(0xffffffff)
                 .setFont(createFont("orbitron",15));
  
  portList = cp5.addDropdownList("portList")
                .setPosition(55, 70)
                .setSize(200, 120)
                .setBackgroundColor(color(190))
                .setItemHeight(20)
                .setBarHeight(20);
                //.setFont(createFont("avenir",12));
                
  for (int i=0;i<Serial.list().length;i++) {
    portList.addItem("["+i+"] "+ Serial.list()[i], i);
  }
  portList.setColorBackground(color(60));
  portList.setColorActive(color(255, 128));
  
  
  // textArea
  promptLabel = cp5.addTextlabel("promptLabel")
                 .setText("Status Prompt: ")
                 .setPosition(300,50)
                 .setColorValue(0xffffffff)
                 .setFont(createFont("orbitron",15));
  
  prompt = cp5.addTextarea("prompt")
                  .setPosition(305,70)
                  .setSize(350,200)
                  .setFont(createFont("courier",12))
                  .setLineHeight(14)
                  .setColor(color(0,255,0))
                  .setColorBackground(color(0))
                  .setColorForeground(color(255,100));
                  
  soutLabel = cp5.addTextlabel("soutLabel")
                 .setText("Serial Output: ")
                 .setPosition(300,300)
                 .setColorValue(0xffffffff)
                 .setFont(createFont("orbitron",15));
                  
  serial_window = cp5.addTextarea("sout")
                     .setPosition(305,320)
                     .setSize(350,200)
                     .setFont(createFont("courier",12))
                     .setLineHeight(14)
                     .setColor(color(0,255,0))
                     .setColorBackground(color(0))
                     .setColorForeground(color(255,100));
                     
  senvalLabel = cp5.addTextlabel("senvalLabel")
                   .setText("Sensor Input")
                   .setPosition(743,410)
                   .setColorValue(0xffffffff)
                   .setFont(createFont("orbitron",15));
                   
   cp5.addTextlabel("motor")
      .setText("Select Motor: ")
      .setPosition(695,50)
      .setColorValue(0xffffffff)
      .setFont(createFont("orbitron",15));
      
    cp5.addTextlabel("sensor")
      .setText("Select Sensor: ")
      .setPosition(695,170)
      .setColorValue(0xffffffff)
      .setFont(createFont("orbitron",15));
                  
  text = "Please Select Serial Port.";
  sout = "**********Serial output from Arduino**********\n";
}

void draw() {
  background(192);
  prompt.setText(text);
  if(firstContact==true) {
    prompt.setText(text+"\n"+which_motor+which_sensor+"***-:Sensor Value: " + sensor_value + "\n" + sensorPrompt + realSensorVal + Unit);
  }
  serial_window.setText(sout);
  
  noStroke();
  fill(0,45,90);
  arc(800, 425, 200, 200, HALF_PI+QUARTER_PI, 2*PI+QUARTER_PI, PIE);
  
  fill(0, 116, 217);
  arc(800, 425, 200, 200, HALF_PI+QUARTER_PI, HALF_PI+QUARTER_PI+ 1.5*PI*sensor_value/1024, PIE);
  
  fill(192);
  ellipse(800, 425, 150, 150);
}

void serialEvent(Serial myPort) {
  arduinoResponse = myPort.readStringUntil('\n');
  if (arduinoResponse != null) {
    if(firstContact==false) {
      firstContact = true;
      text = text + "- Serial Connection Established.\n" 
      ;
    } else {
      sout = sout + arduinoResponse;
      String[] splitted = split(arduinoResponse, '\r');
      sensor_value = int(splitted[0]);
      println(sensor_value);
      switch(sensorUsing) {
        case 0:
          realSensorVal = (float)sensor_value/1023*270;
          break;
        case 1:
          realSensorVal = (6762/(sensor_value-9))-4;
          if(realSensorVal > 80 || realSensorVal < 10) {
            realSensorVal = -1;
          }  
          break;
        case 2:
          realSensorVal = sensor_value/2;
          break;
        case 3:
          realSensorVal = ((sensor_value*(5.0/1023.0))*1000-500)/10;
          break;
      }
    }    
  }
} 

void controlEvent(ControlEvent theEvent) {
  if(theEvent.isAssignableFrom(Textfield.class)) {
    println("controlEvent: accessing a string from controller '"
            +theEvent.getName()+"': "
            +theEvent.getStringValue()
            );
    String val = theEvent.getStringValue();
    if(firstContact == true) {
      myPort.write(val);
    }
  } else if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    println("event from group : "+theEvent.getGroup().getValue()+" from "+theEvent.getGroup());
  } else if (theEvent.isController()) {   
    
    println("event from controller : "+theEvent.getController().getValue()+" from "+theEvent.getController());
    if(theEvent.getController().getName()=="portList") {
      int portNum = (int)theEvent.getController().getValue();
      
      if(firstContact==true) {
        myPort.stop();
        firstContact=false;
      }
      portName = Serial.list()[portNum];
      
      try {
        myPort = new Serial(this, portName, 9600);
        myPort.bufferUntil('\n');
        text = "Waiting for connection with Arduino.\n";
      } catch(RuntimeException e) {
        text += "- Serial connection failed.\n";
        firstContact = false;
      }
    } else if (theEvent.getController().getName()=="stepper" && firstContact==true) {
      which_motor = "**Current Motor: Stepper\n";
      myPort.write("m1");
    } else if (theEvent.getController().getName()=="dc_pos" && firstContact==true) {
      which_motor = "**Current Motor: DC Motor (Position)\n";
      myPort.write("m2");
    } else if (theEvent.getController().getName()=="dc_vel" && firstContact==true) {
      which_motor = "**Current Motor: DC Motor (Velocity)\n";
      myPort.write("m3");
    } else if (theEvent.getController().getName()=="servo" && firstContact==true) {
      which_motor = "**Current Motor: Servo\n";
      myPort.write("m0");
    } else if (theEvent.getController().getName()=="IR" && firstContact==true) {
      which_sensor = "**Current Sensor: IR\n";
      sensorUsing = 1;
      sensorPrompt = "***-: IR Distance: ";
      Unit = " cm";
      myPort.write("s1");
    } else if (theEvent.getController().getName()=="sonar" && firstContact==true) {
      sensorUsing = 2;
      which_sensor = "**Current Sensor: Sonar\n";
      sensorPrompt = "***-: Sonar Distance: ";
      Unit = " in";
      myPort.write("s2");
    } else if (theEvent.getController().getName()=="temp" && firstContact==true) {
      sensorUsing = 3;
      which_sensor = "**Current Sensor: temp\n";
      sensorPrompt = "***-:Temperture: ";
      Unit = " degree Celcius";
      myPort.write("s3");
    } else if (theEvent.getController().getName()=="pot" && firstContact==true) {
      sensorUsing = 0;
      which_sensor = "**Current Sensor: Pot\n";
      sensorPrompt = "***-:Angle: ";
      Unit = " degree";
      myPort.write("s0");
    }
    //println(which_motor);
    //println(which_sensor);
  }
}