// 교량관리시스템

//RFID 설정
#include <MFRC522DriverSPI.h>
#include <MFRC522v2.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
class MFRC522DriverPinSimple sda_pin(53); //RFID포트 53,52,51,50
class MFRC522DriverSPI driver {sda_pin};
class MFRC522 mfrc522 {driver};
const String MASTER_CARD_UID {String("D78B4019")}; //관리자카드

//DHT11(온습도센서) 설정
#include "DHT.h"
const uint8_t TEMP_HUMID_PIN {A0}; // 온습도센서 포트 A0
class DHT dht(TEMP_HUMID_PIN, 11);

//LCD 설정
#include <LiquidCrystal_I2C.h>
class LiquidCrystal_I2C lcd (0x27, 16, 2);

//스탭퍼모터 설정
#include <Stepper.h>
const uint16_t STEP_PER_REVOLUTION {2048U};
class Stepper stepping(STEP_PER_REVOLUTION, 36,37,38,39);//IN4 IN2 IN3 IN1

//HC-SR04(초음파센서) 설정
#include "HCSR04.h"
enum ULTRASONIC {// 초음파센서 포트 6,7
  TRIGGER = 6U,
  ECHO
};
class UltraSonicDistanceSensor ultra_sonic(TRIGGER,ECHO);

//핀설정
//INPUT
const uint8_t WATER_PIN {A1}; // 수위감지센서
const uint8_t SW_PIN {3U}; // 비상정지버튼 포트 3(인터럽트)
const uint8_t EC_PIN {30U}; // 비상정지해제 버튼 포트 30

//OUTPUT
const uint8_t SERVO_PIN {5U}; // 서보모터 -> 아날로그출력
const uint8_t RELAY_PIN {8U}; // 전원 릴레이스위치 포트 8
const uint8_t BUZZ_PIN {9U}; // 피에조 부저 포트 9 -> 톤 출력
enum ShiftRegisterPIN { // 시프트레지스터 포트 24~26
  DS_PIN = 24U,
  LATCH_PIN,
  CLOCK_PIN
};
enum RGB{
  RED = 10U,
  GREEN,
  BLUE
};
 
//전역변수설정
const uint16_t tone_first {500U}; // 부저 소리 주파수 설정 1
const uint16_t tone_second {700U}; // 부저 소리 주파수 설정 2
volatile bool login_check {false}; // 관리자 체크인여부
volatile int16_t loop_count {0}; //루프 카운터
volatile bool emergency_mode {false}; // 긴급정지모드 여부
volatile bool dht11_state {false}; //온습도센서
volatile float distance {-1.0}; //초음파센서 측정값
volatile uint16_t water_value {1024}; // 수위센서 측정값
volatile bool ship_close {false}; // 배 통과 대기
volatile bool gate_state {false}; // 게이트바 상태
volatile bool no_entry {true}; // 출입금지 여부

//고정값 지정
#define ST_DELAY 100UL // 기본딜레이 -> 현재 최소딜레이 동작 200ms
#define MAX_LOOP 50UL // 최대 루프 수
#define WARNING_DIST 20 // 물체접근위험 기준거리

//커스텀 함수 목록
void initAll(); // 시스템초기화(전원공급릴레이 및 RFID제외)
void myShift(uint8_t leds);// 시프트레지스터 제어 함수
void lcdClear(); // LCD 정리 함수
void openBridge(); // 교량개방처리 함수
void noEntry(bool open); // 출입금지처리 함수
void rgbLED(String color_text); //RGB LED 색상제어 함수
void emergencyStop(); //비상정지(인터럽트) 처리 함수

void setup() {
  Serial.begin(115200UL);
  mfrc522.PCD_Init();
  stepping.setSpeed(14); // 스탭퍼모터 속도설정

  //핀 모드 설정
  pinMode(WATER_PIN, INPUT);
  pinMode(EC_PIN,INPUT);
  pinMode(SW_PIN,INPUT);

  pinMode(SERVO_PIN,OUTPUT);
  pinMode(RELAY_PIN,OUTPUT);
  pinMode(BUZZ_PIN,OUTPUT);
  pinMode(DS_PIN,OUTPUT);
  pinMode(LATCH_PIN,OUTPUT);
  pinMode(CLOCK_PIN,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);
  lcd.init(); // LCD 초기화
  lcd.noBacklight();
}

void loop() {
  if (!login_check){//관리자 부재시
    if(!mfrc522.PICC_IsNewCardPresent()) return;
    if(!mfrc522.PICC_ReadCardSerial()) return;
    String tagID = "";
    for(uint8_t i {0u} ; i < 4 ; ++i){
      tagID += String(mfrc522.uid.uidByte[i],HEX);
    }
    tagID.toUpperCase();
    mfrc522.PICC_HaltA();
    if( tagID == MASTER_CARD_UID)
    {
      Serial.println(F("업무를 시작합니다."));
      login_check = true;
    }
    else {
      Serial.println(F("잘못된 카드입니다."));
      return;
    }
  }
  else {// 관리자 체크인
    digitalWrite(RELAY_PIN,HIGH); // 전원연결
    delay(100UL); //서지 현상 방지
    initAll(); // 시스템초기화
    
    //변수선언
    float temperature {-1.0};
    float humidity {-1.0};
    uint8_t water_level {0};

    //업무시작
    for(;;){
      if(loop_count >= MAX_LOOP){
        loop_count = 0;
      }
      if(!emergency_mode){//긴급정지상태가 아닐 시
        // 1. 센서확인 : 온습도센서, 초음파센서, 수위감지센서
          //온습도센서
        if(loop_count%(static_cast<int>(500/ST_DELAY)) == 0){//500ms마다 동작
          dht11_state = dht.read();
          if(dht11_state){
            temperature = dht.readTemperature();
            humidity = dht.readHumidity();

            lcdClear();
            lcd.print(String(F("TEMP: ")) + String(temperature) + String(F(" C")));
            lcd.setCursor(0,1);
            lcd.print(String(F("HUMID: ")) + String(humidity) + String(F(" %")));
            lcd.setCursor(0,0);
            
            //초음파센서
            distance = ultra_sonic.measureDistanceCm(temperature);
            Serial.println(String(F("Distance : ")) + String(distance));
          }
        }

          //수위감지센서
        if(loop_count%(static_cast<int>(2000/ST_DELAY)) == 0){//2000ms마다 동작
          water_value = analogRead(WATER_PIN);
          if(0 <= water_value and water_value < 1024){
            water_level = map(water_value,0,1023,0,4);
            Serial.println(String(F("WaterLv : ")) + String(water_level));
          }
        }
        // 2. LED점등
        if(loop_count%(static_cast<int>(200/ST_DELAY)) == 0){//200ms마다 동작
          ledUpdate();
        }
        // 3. 동작처리
        if ( 400.0 < distance or distance < 0.0F ) {}
        else {
          if(distance <= WARNING_DIST){ // 1) 배 접근 시 -> 교량 개방
            openBridge();
          }
        }
        if(water_level == 2) {// 2) 수위 감지 -> 출입금지
          noEntry(false);
        }
        else {
          noEntry(true);
        }
        //RFID 체크
        if(mfrc522.PICC_IsNewCardPresent()){
          if(mfrc522.PICC_ReadCardSerial()){
            String tagID = "";
            for(uint8_t i {0u} ; i < 4 ; ++i){
              tagID += String(mfrc522.uid.uidByte[i],HEX);
            }
            tagID.toUpperCase();
            mfrc522.PICC_HaltA();
            if( tagID == MASTER_CARD_UID){
              //업무종료
              login_check = false;
              Serial.println(F("오늘의 업무를 종료합니다."));
              // 전원 차단 전 처리
              noEntry(false);
              Serial.println(F("곧 전원이 차단됩니다."));
              delay(ST_DELAY*50);
              myShift(B00000000);
              lcdClear(); // LCD 출력 초기화
              lcd.noBacklight();
              digitalWrite(RELAY_PIN,LOW);
              break;
            }
            else
              Serial.println(F("잘못된 카드입니다."));
          }
        }
      }
      else{//긴급정지 시
      }
      ++loop_count;
      delay(ST_DELAY);
    }
    detachInterrupt(digitalPinToInterrupt(SW_PIN));
  }
}
void rgbLED(String color_text){
  if(color_text == "red"){
    analogWrite(RED, 255);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
  }
  else if(color_text == "green"){
    analogWrite(RED, 0);
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 0);
  }
  else if(color_text == "blue"){
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 255);
  }
  else if(color_text == "yellow"){
    analogWrite(RED, 255);
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 0);
  }
  else if(color_text == "black"){
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
  }
}

void initAll(){ // 시스템 초기화 함수
  Serial.println(F("System Initializing..."));
  dht.begin(); // DHT11 초기화
  Serial.println(F("DHT11 init"));
  lcd.backlight(); // LCD 백라이트 ON
  lcdClear(); // LCD 출력 초기화
  for( int i {0}; i < 3 ;++i){// LED 동작확인
    myShift(B00000000);
    delay(ST_DELAY*10);
    myShift(B11111111);
    delay(ST_DELAY*10);
  }
  myShift(B00000000); // LED 초기화
  EIFR |= ( 1 << INTF1 ); // 비활성화 중 호출 초기화
  attachInterrupt(digitalPinToInterrupt(SW_PIN), emergencyStop, RISING); //인터럽트 설정
  rgbLED("black");  // RGB LED off
  noEntry(true); // Gate open
  Serial.println(F("System Initialized."));
}

void lcdClear(){ // LCD 출력 초기화 함수
  lcd.home();
  lcd.print(F("                "));
  lcd.setCursor(0,1);
  lcd.print(F("                "));
  lcd.home();
}

void ledUpdate(){
  // 1)관리자 출퇴근
  // 2,3,4)온습도, 초음파, 수위감지 센서 동작
  // 5)배 접근 감지 
  // 6,7)출입금지등*2
  // 8)비상정지 -> 인터럽트시에만 활용됨
  static uint8_t leds = B00000000;
  if(login_check) leds |= B10000000;
  else leds &= ~B10000000;

  if(dht11_state) leds |= B01000000;
  else leds &= ~B01000000;

  if(distance>=0) leds |= B00100000;
  else leds &= ~B00100000;

  if(water_value<1024) leds |= B00010000;
  else leds &= ~B00010000;

  if(ship_close) leds |= B0001000;
  else leds &= ~B00001000;

  if(!gate_state) leds |= B0000100;
  else leds &= ~B0000100;

  if(no_entry) leds |= B0000010;
  else leds &= ~B0000010;

  myShift(leds);
}

void myShift(uint8_t leds){// 시프트레지스터 제어 함수
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DS_PIN,CLOCK_PIN, MSBFIRST, leds);
  digitalWrite(LATCH_PIN, HIGH);
}

void openBridge(){ // 교량 개방 처리 함수
  const uint16_t new_step = static_cast<uint16_t>(STEP_PER_REVOLUTION/64);
  Serial.println("Bridge OPEN");
  ship_close = true;
  rgbLED("red");
  noEntry(false);
  for(int i {0};i < 64;++i){ // 교량 개방
    stepping.step(new_step);
    //if(i%2 == 0) tone(BUZZ_PIN, tone_first);
    //else tone(BUZZ_PIN, tone_second);
  }
  rgbLED("green");
  float dist {-1.0};
  uint8_t pass_cnt = 0;
  for(uint16_t i {0};;++i){ //선박 통과 시
    if(i == 65534) i = 0;
    //if(i%2 == 0) tone(BUZZ_PIN, tone_first);
    //else noTone(BUZZ_PIN);
    dist = ultra_sonic.measureDistanceCm(dht.readTemperature());
    if(dist > WARNING_DIST or dist > 400.0 or dist < 0.0) ++pass_cnt;
    if(pass_cnt > 10){
      distance = -1.0;
      break;
    }
    delay(ST_DELAY*5);
  }
  rgbLED("yellow");
  //tone(BUZZ_PIN, tone_second);
  delay(ST_DELAY*30);
  rgbLED("red");
  //noTone(BUZZ_PIN);
  ship_close = false;
  ledUpdate();
  delay(ST_DELAY*10);
  Serial.println("Bridge CLOSE");
  for(int i {63};i >= 0;--i){ // 교량 폐쇄
    stepping.step(-new_step);
    //if(i%2 == 0) tone(BUZZ_PIN, tone_first);
    //else tone(BUZZ_PIN, tone_second);
  }
  noTone(BUZZ_PIN);
  rgbLED("black");
  noEntry(true);
}

void noEntry(bool open){ // 출입금지 처리 함수
  if(open && !gate_state){
    Serial.println("Gate OPEN");
    no_entry = true;
    ledUpdate();
    for(int i {0};i<128;i+=2){
      analogWrite(SERVO_PIN,i);
      delay(ST_DELAY);
    }
    Serial.println("통행가능");
    delay(ST_DELAY);
    gate_state = true;
    no_entry = false;
    ledUpdate();
  }
  else if (!open && gate_state){
    Serial.println("Gate CLOSE");
    no_entry = true;
    ledUpdate();
    for(int i {127};i>=0;i-=2){
      analogWrite(SERVO_PIN,i);
      delay(ST_DELAY);
    }
    delay(ST_DELAY);
    gate_state = false;
    ledUpdate();
  }
}

void emergencyStop() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if(interrupt_time - last_interrupt_time > 300) // 비상정지버튼 입력이 연속적으로 인식되는 현상 방지
  {
    if(!emergency_mode){
      for (int i {0}; i < 3 ;++i)
        Serial.println(F("긴급 정지"));
      myShift(B00000001);
      emergency_mode = true;
    }
    else{
      Serial.println(F("복구되었습니다."));
      myShift(B00000000);
      emergency_mode = false;
    }
  }
  last_interrupt_time = interrupt_time;
}