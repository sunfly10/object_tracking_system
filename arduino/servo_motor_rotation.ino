//X축 Y축 서보모터 제어(라이브러리 활용) + 시리얼 통신을 통한 각도 주고 받기
//최종 수정일: 26.2.16(주석 간단하게)

#include <Servo.h>

Servo servo1;
Servo servo2;    
int currentangle_x = 90;
int currentangle_y = 90;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);
  servo1.write(currentangle_x);  
  servo2.attach(8);
  servo2.write(currentangle_y);
}

void loop() {
  if (Serial.available()) { 
    String input = Serial.readStringUntil('\n'); //\n으로 publish했고 serial 통신 끝나면 그 자리 유지
    input.trim();
    if (input.length() == 0) return;

    if (input.startsWith("receive_x")){
       Serial.println(180-currentangle_x);
       return; 
    }
    
    if (input.startsWith("x:")){
      int val = input.substring(2).toInt();
      if (val >=0 && val <= 180){
      int targetangle = 180 - val;
      servo1.write(targetangle);
      int diff = abs(targetangle - currentangle_x);
      delay(2 * diff);  //이동 시간
      currentangle_x = targetangle;
      }
      return;
    }
    
    if (input.startsWith("receive_y")){
       Serial.println(180-currentangle_y); 
       return;
    }
    
    if (input.startsWith("y:")){
      int val = input.substring(2).toInt();
      if (val >=0 && val <= 180){
      int targetangle = 180 - val;
      servo2.write(targetangle);
      int diff = abs(targetangle - currentangle_y);
      delay(2 * diff);  //이동 시간
      currentangle_y = targetangle;
     }
     return;
    }
  }
}
