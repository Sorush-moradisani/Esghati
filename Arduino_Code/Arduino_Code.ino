#include <SoftwareSerial.h>
#include <VarSpeedServo.h> 

SoftwareSerial mySerial(4, 3); 

VarSpeedServo  Rwheel;
VarSpeedServo  Lwheel;

int VSPEED = 5 , HSPEED = 10 , MSPEED = 50, MACCEL = 125;
int MAX_VSPEED = 10 , MAX_HSPEED = 20 , MAX_MSPEED = 255, MSTOP = 1500, MSTOPdeg = 90;

String command;
bool commandEnd=false;
char inByte;
int arg = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySerial.begin(9600);

  Rwheel.attach(6);
  delay(100);
  Rwheel.write(MSTOP);
  delay(300);

  Lwheel.attach(5);
  delay(100);
  Lwheel.write(MSTOP);
  delay(300);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available() > 0) {

      if(commandEnd){
      command="";
      commandEnd=false;
    }
      inByte = mySerial.read();
      if (inByte != '\n') {
        if (inByte == '=') {
          arg = mySerial.parseInt();
          //Serial.println(arg);      
        } else {
          command += inByte;
        }
      }else{
        commandEnd=true;
        Serial.println(command);
        execute(command, arg);
      }
  }
    
}

void execute(String command, int arg){
if(compareString(command, "l-rotate")){
      Rwheel.write(MSTOPdeg-MSPEED,MACCEL);
      Lwheel.write(MSTOPdeg-MSPEED,MACCEL);
      command="";
    }else if(compareString(command, "r-rotate")){
      Rwheel.write(MSTOPdeg+MSPEED,MACCEL);
      Lwheel.write(MSTOPdeg+MSPEED,MACCEL);
      command="";
    }else if(compareString(command, "f-forward")){
      Rwheel.write(MSTOPdeg-MSPEED, MACCEL);
      Lwheel.write(MSTOPdeg+MSPEED, MACCEL);
      command="";
    }else if(compareString(command, "f-left")){
      Rwheel.write(MSTOPdeg-MSPEED,MACCEL);
      Lwheel.write(MSTOP);
      command="";
    }else if(compareString(command, "f-right")){
      Rwheel.write(MSTOP);
      Lwheel.write(MSTOPdeg+MSPEED,MACCEL);
      command="";
    }else if(compareString(command, "b-backward")){
      Rwheel.write(MSTOPdeg+MSPEED,MACCEL);
      Lwheel.write(MSTOPdeg-MSPEED,MACCEL);
      command="";
    }else if(compareString(command, "b-right")){
      Rwheel.write(MSTOPdeg+MSPEED,MACCEL);
      Lwheel.write(MSTOP);
      command="";
    }else if(compareString(command, "b-left")){
      Rwheel.write(MSTOP);
      Lwheel.write(MSTOPdeg-MSPEED,MACCEL);
      command="";
    }else if(compareString(command, "stop")){
      Rwheel.write(MSTOP);
      Lwheel.write(MSTOP);
      command="";
    }
    command="";
}

boolean compareString(String a, String b) {
  if (a.length() != b.length() + 1) {
    return false;
  }
  for (int i = 0; i < a.length() - 1; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}
