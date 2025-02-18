#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 3); 

String command;
bool commandEnd=false;
char inByte;
int arg = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySerial.begin(9600);
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
        Serial.print(command);
        Serial.print("=");
        Serial.println(arg);
      }
  }

  else if(command == "r-rotate"){
      analogWrite(6, 22);
      analogWrite(5, 22);
      command="";
    }else if(command == "l-rotate"){
      analogWrite(6, 254);
      analogWrite(5, 254);
      command="";
    }else if(command == "f-forward"){
      analogWrite(6, 254);
      analogWrite(5, 22);
      command="";
    }else if(command == "f-right"){
      analogWrite(6, 0);
      analogWrite(5, 22);
      command="";
    }else if(command == "f-left"){
      analogWrite(6, 254);
      analogWrite(5, 188);
      command="";
    }else if(command == "b-backward"){
      analogWrite(6, 22);
      analogWrite(5, 254);
      command="";
    }else if(command == "b-left"){
      analogWrite(6, 22);
      analogWrite(5, 0);
      command="";
    }else if(command == "b-right"){
      analogWrite(6, 0);
      analogWrite(5, 254);
      command="";
    }else if(command == "stop"){
      analogWrite(6, 0);
      analogWrite(5, 0);
      command="";
    }
}
