//#define in11  7
//#define in21  4
//#define in31  2
//#define in41  3
//#define in12  8
//#define in22  11
//#define in32  12
//#define in42  13

#define P1 32
#define D1 33

#define P2 25
#define D2 26


int speed = 150;
int rate = 5;

void gradual_speed(int final_speed, int rate) {
  for (int speed = 0; speed < final_speed; speed++) {
    analogWrite(P1, speed);
    analogWrite(P2, speed);
    Serial.println(speed);
    delay(rate);
  }
  analogWrite(P1, final_speed);
  analogWrite(P2, final_speed);
}

void setup() {
  Serial.begin(115200);

  pinMode(P1, OUTPUT);
  pinMode(D1, OUTPUT);

  pinMode(P2, OUTPUT);
  pinMode(D2, OUTPUT);
  
  
}

void loop() {
  while (Serial.available() > 0) {
    // Read the incoming byte
    String cmd = Serial.readStringUntil('\n');
    Serial.println(cmd);
    
    // Parse the received string
    int delimiterIndex = cmd.indexOf(',');
    String leftPWM = cmd.substring(0, delimiterIndex);
    String rightPWM = cmd.substring(delimiterIndex + 1);

//    int leftPWM;
//    int rightPWM;
    // int leftPWMValue;
    // int rightPWMValue;

    int leftPWMValue = leftPWM.toInt();
    int rightPWMValue = rightPWM.toInt();

       if(leftPWMValue>0 && rightPWMValue>0 ){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        gradual_speed(speed, rate);
        break; 


     } 
   else if(leftPWMValue<0 && rightPWMValue<0){
        

        
        digitalWrite(D1, LOW);
        digitalWrite(D2, LOW);
        gradual_speed(speed, rate);
        break;  
   }
   else if(leftPWMValue<0 && rightPWMValue>0){
    //RIGHT
        digitalWrite(D1, HIGH);
        digitalWrite(D2, LOW);

        gradual_speed(speed, rate);
        break;
   }
   else if(leftPWMValue>0 && rightPWMValue<0){
    //LEFT
        digitalWrite(D1, LOW);
        digitalWrite(D2, HIGH);

        gradual_speed(speed, rate);
        break;
   }

   else{
        analogWrite(P1, 0);
        analogWrite(P2, 0);
        break;
   }





















//
//
// if (left_PWM == 100 && right_PWM == 100) {
//     leftPWMValue = 100;
//     rightPWMValue = 100;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
// }
// if (left_PWM == 0 && right_PWM == 0) {
//     leftPWMValue = 0;
//     rightPWMValue = 0;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
// }
//     if (left_PWM == 200) {
//         leftPWMValue = 200;
//         rightPWMValue = 0;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     } else if (left_PWM == 300) {
//         leftPWMValue = 255;
//         rightPWMValue = -100;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,LOW);
//       digitalWrite(m3_dir,LOW);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     } else if (left_PWM == 400) {
//         leftPWMValue = 400;
//         rightPWMValue = -200;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,LOW);
//       digitalWrite(m3_dir,LOW);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     }

//     if (right_PWM == 200) {
//         leftPWMValue = 0;
//         rightPWMValue = 200;
//       digitalWrite(m1_dir,HIGH);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,HIGH);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     } else if (right_PWM == 300) {
//         leftPWMValue = -100;
//         rightPWMValue = 255;
//       digitalWrite(m1_dir,LOW);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,LOW);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     } else if (right_PWM == 400) {
//         leftPWMValue = -200;
//         rightPWMValue = 255;
//       digitalWrite(m1_dir,LOW);
//       digitalWrite(m2_dir,HIGH);
//       digitalWrite(m3_dir,HIGH);
//       digitalWrite(m4_dir,LOW);
//       leftPWMValue=abs(leftPWMValue);
//       rightPWMValue=abs(rightPWMValue);
//       analogWrite(m2_pwm, leftPWMValue);
//       analogWrite(m3_pwm, leftPWMValue);
//       analogWrite(m1_pwm, rightPWMValue);  
//       analogWrite(m4_pwm, rightPWMValue);
//     }



//    if(leftPWMValue>=255){
//      leftPWMValue=255;
//    }
//    if(leftPWMValue<=-255){
//      leftPWMValue=-255;
//    }
//    if(rightPWMValue>=255){
//      rightPWMValue=255;
//    }
//    if(rightPWMValue<=-255){
//      rightPWMValue=-255;
//    }
//    if(-255<rightPWMValue<255){
//      rightPWMValue=rightPWMValue;
//    }
//    if(-255<leftPWMValue<255){
//      leftPWMValue=leftPWMValue;
//    }

//    Serial.print(leftPWMValue);
//    Serial.print(", ");Serial.println(rightPWMValue);

   
  //  else{
  //    analogWrite(m2_pwm, 0);
  //    analogWrite(m1_pwm, 0);  
  //  }


    // Print the PWM values to the Serial Monitor
    // Serial.print("Left PWM: ");
    // Serial.print(leftPWMValue);
    // Serial.print(", Right PWM: ");
    // Serial.println(rightPWMValue);

    // Set the PWM values to the motors
    
    //analogWrite(leftMotorPin, leftPWMValue);
    //analogWrite(rightMotorPin, rightPWMValue);
  }
}
