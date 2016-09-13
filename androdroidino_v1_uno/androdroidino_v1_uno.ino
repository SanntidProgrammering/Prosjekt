

#include <Wire.h>
#include <Servo.h>
#include <PixyI2C.h> 


// setup servo
Servo leftServo;
Servo rightServo;

// servo vinkler
int minAngleLeft = 0;
int maxAngleLeft = 180;
int minAngleRight = 0;
int maxAngleRight = 180;

//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

// distanse sensor oppsett og kalibrering
const int DIST_SENSOR_PIN = 15;
int sensorValueRAW;
byte sensorValue;
int rawLow = 0;
int rawHigh  = 1023;
byte valueLow = 4;
byte valueHigh = 30;

// code below includes the variables to PIXY CAMERA
PixyI2C pixy;
int xValue;
int yValue;

char outputData[6];
char inputData[6];

boolean start = true;

// debugger bit
boolean debug = true;

char rx_byte = 0;

//________________________________________________________________
//___________________________ SETUP ______________________________
void setup() 
{
  int i;
  for(i=4;i<=7;i++){
    pinMode(i, OUTPUT);
  }
  Serial.begin(19200);      //Set Baud Rate
  
  Wire.begin(10);                // join i2c bus with address #10
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);
  if(debug) Serial.println("Wire complete");
  leftServo.attach(10); 
  rightServo.attach(11);
  
  //start pixy camera
  pixy.init();
  if(debug) Serial.println("setup complete");
}
//_____________________________________________________________________________
//___________________ LOOP_____________________________________________________
void loop(){
    if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read(); 
    inputData[0] = rx_byte-'0';
    Serial.println(inputData[0]);
    dataInputHandler();
    }  
}

//behandling av data
void dataInputHandler(){
  if(debug){
    Serial.println("datainputhandler");
    for(int i = 0; i < sizeof(inputData); i++){
      Serial.print(inputData[i]);
      Serial.print(",");
      }
  }
  
  // kontrollering av armer
  if (bitRead(inputData[3],0)){
    leftArmOut();
  }
  else if(!bitRead(inputData[3],0)) {
    //leftArmIn();
  }
  
  if (bitRead(inputData[3],1)) {
    rightArmOut();
  }
  else if(!bitRead(inputData[3],1)) {
    //rightArmIn();
  }
  
  // startbit for kjÃ¸ring
  //start = bitRead(inputData[3],3);
 // if(debug) Serial.println("startbit  " + start);
  
  // aktiver kommandoer hvis start er aktiv
  if(start){
    
    int state = inputData[0];
    if(debug){
      Serial.println("state:");
      Serial.print(state);
      }
    switch(state){
      case 0:
        if(debug) Serial.println("case 0");
        stopp();
        break;
      
      case 1:
        if(debug) Serial.println("case 1");
        advance(inputData[1],inputData[2]);
        break;
      
      case 2:
        if(debug) Serial.println("case 2");
        back_off(inputData[1],inputData[2]);
        break;
      
      case 3:
        if(debug) Serial.println("case 3");
        turn_L(inputData[1],inputData[2]);
        break;
      
      case 4:
        if(debug) Serial.println("case 4");
        turn_R(inputData[1],inputData[2]);
        break;
        
       default:
         Serial.println("default");
    }
    // stopp bil hvis startbit ikke er aktiv
  } else {
     if(debug) Serial.println("stopp bil fra dataInputHandler");
     stopp();
  } 
      
  
}

// endre felt i inputdata
void setInputByte(byte b, int i){
  if(0<i< sizeof(inputData)){
  inputData[i] = b;
  if(debug) Serial.println("input data endret  data[");
            Serial.print(i);
            Serial.print("] ny verdi: ");
            Serial.print(b);
  }
}
  

//behandling av data til odroid
void dataOutputHandler(){
  readDistSensor();
  pixyCamera();
  outputData[0] = lowByte(xValue);
  outputData[1] = highByte(xValue);
  outputData[2] = lowByte(yValue);
  outputData[3] = highByte(yValue);
  outputData[4] = sensorValue;
  outputData[5] = 0;
  if(debug) {
    Serial.println("output data endret");
    for(int i=0; i < sizeof(outputData); i++){
      Serial.println("byte[");
      Serial.print(i);
      Serial.print("] ny verdi: ");
      Serial.print(outputData[i]);
    }
  }
}
//___________________________________________________________________________
//__________________________ I2C kommunikasjon_______________________________


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  if(debug) Serial.println("requestEvent aktivert");
  
  dataOutputHandler();
  Wire.write(outputData); // respond with message of 6 bytes
  // as expected by master
}

void receiveEvent(int nrOfBytes){
  if(debug) Serial.println("ReceiveEvent aktivert");
  
  int i = 0;
  while (1<Wire.available()) {
    setInputByte(Wire.read(),i);
    i++;
    if (i > sizeof(inputData)) break;
  }
  //behandler data fra i2c bus
  dataInputHandler();
}

//___________________________________________________________
//____________ kontrollering av servo armer__________________

void leftArmOut() {                    // venstre arm ut
  leftServo.write(maxAngleLeft);
  if(debug) Serial.println("leftArmOut");
}
void leftArmIn(){                      // venstre arm inn
  leftServo.write(minAngleLeft);
  if(debug) Serial.println("leftArmIn");
}
void rightArmOut(){                  // hÃ¸yre arm ut
  rightServo.write(maxAngleRight);
  if(debug) Serial.println("rightArmOut");
}  
void rightArmIn(){                   // hÃ¸yre arm inn
  rightServo.write(minAngleRight);
  if(debug) Serial.println("rightArmIn");
}

//____________________________________________________________
//____________________ kjÃ¸ring av bil ________________________
void stopp()                         //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);
  if(debug) Serial.println("stopp");  
}   
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);                //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  if(debug) Serial.println("advance");
            Serial.print(a);
            Serial.print("  ");
            Serial.print(b);
}  
void back_off (char a,char b)        //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
  if(debug) Serial.println("back_off:");
            Serial.print(a);
            Serial.print("  ");
            Serial.print(b);
}
void turn_L (char a,char b)           //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  if(debug) Serial.println("turn_L:");
            Serial.print(a);
            Serial.print("  ");
            Serial.print(b);
}
void turn_R (char a,char b)           //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
  if(debug) Serial.println("turn_R:");
            Serial.print(a);
            Serial.print("  ");
            Serial.print(b);
}

//_____________________________________________________________________
//________________________Distanse sensor______________________________
void readDistSensor(){
  sensorValueRAW = analogRead(DIST_SENSOR_PIN);
  sensorValue = map(sensorValueRAW,rawLow,rawHigh,valueLow,valueHigh);
  if(debug) Serial.println("readDistsensor, raw: ");
            Serial.print(sensorValueRAW);
            Serial.print(" value: ");
            Serial.print(sensorValue);
}

//_______________________________________________________________________
//________________________Pixy camera code_______________________________

void pixyCamera()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int xValue;
  int yValue;

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        xValue = pixy.blocks[j].x;
        yValue = pixy.blocks[j].y;
      }
    }
  }
}



