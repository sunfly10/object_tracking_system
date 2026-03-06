// X축 Y축 서보모터 제어(라이브러리 활용) + 시리얼 통신을 통한 각도 주고 받기
// 최종 수정일: 26.2.16

#include <Servo.h>

Servo servo1; // X축 제어용 서보모터 (Pin 9)
Servo servo2; // Y축 제어용 서보모터 (Pin 8)
int currentangle_x = 90; // X축 초기 중심각 설정
int currentangle_y = 90; // Y축 초기 중심각 설정

void setup() {
  Serial.begin(9600); // 시리얼 통신 속도 설정 (9600 bps)
  
  servo1.attach(9);
  servo1.write(currentangle_x);  // X축 서보모터 초기 위치 구동
  
  servo2.attach(8);
  servo2.write(currentangle_y);  // Y축 서보모터 초기 위치 구동
}

void loop() {
  // 시리얼 버퍼에 수신된 데이터가 있는지 확인
  if (Serial.available()) { 
    // '\n'을 기준으로 문자열 수신 (이전 통신 상태 유지 목적)
    String input = Serial.readStringUntil('\n'); 
    input.trim(); // 문자열 앞뒤의 불필요한 공백 제거
    if (input.length() == 0) return;

    // 1. X축 현재 각도 피드백 요청 처리
    if (input.startsWith("receive_x")){
       // 하드웨어 장착 방향(역방향)을 고려한 180도 보정값 반환
       Serial.println(180-currentangle_x);
       return; 
    }
    
    // 2. X축 목표 각도 제어 명령 처리
    if (input.startsWith("x:")){
      int val = input.substring(2).toInt(); // "x:" 이후의 제어값 추출
      if (val >=0 && val <= 180){ // 유효한 각도(0~180도) 검증
      int targetangle = 180 - val; // 하드웨어 역방향 보정 연산
      servo1.write(targetangle);
      
      int diff = abs(targetangle - currentangle_x); // 이동해야 할 각도량 계산
      delay(2 * diff);  // 이동 시간 (각도 변화량에 비례하여 안정화 대기)
      currentangle_x = targetangle; // 현재 각도 상태 갱신
      }
      return;
    }
    
    // 3. Y축 현재 각도 피드백 요청 처리
    if (input.startsWith("receive_y")){
       Serial.println(180-currentangle_y); 
       return;
    }
    
    // 4. Y축 목표 각도 제어 명령 처리
    if (input.startsWith("y:")){
      int val = input.substring(2).toInt();
      if (val >=0 && val <= 180){
      int targetangle = 180 - val;
      servo2.write(targetangle);
      
      int diff = abs(targetangle - currentangle_y);
      delay(2 * diff);  // 이동 시간
      currentangle_y = targetangle;
     }
     return;
    }
  }
}
