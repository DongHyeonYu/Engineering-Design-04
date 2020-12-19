#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////
#define SEQ_SIZE 8 //구간별 보정을 위한 구간의 갯수 
float dist_raw2, raw_dist, real_value; //구간별 보정을 위한 변수 선언
// sensor values
float x[8] = {75.0, 115.0, 155.0, 185.0, 215.0 , 240.0 , 265.0, 285.0}; //센서의 실제 측정값
  
// real values
float y[8] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0}; //실제 거리 

////////////////////////////
#define DELAY_MICROS  1500 //필터링을 위한 딜레이 
#define EMA_ALPHA 0.4 //ema필터 적용을 위한 alpha
float filtered_dist; //필터링된 변수 저장을 위한 변수 선언
float ema_dist = 0; //ema필터 적용 후 저장을 위한 변수 선언
float samples_num = 3; //필터링을 위한 샘플링에 사용할 갯수
///////////to filter////////


// Arduino pin assignment
#define PIN_SERVO 10 //10번핀을 SERVO의 핀으로 선언
#define PIN_IR A0  //Analog 0 핀을 적외선 센서의 핀으로 선언

// Framework setting
#define _DIST_TARGET 255 //목표 위치
#define _DIST_MIN 100 //측정 최소거리
#define _DIST_MAX 410 //측정 최대거리

// Servo range
#define _DUTY_MIN 870 //서보 최소 위치
#define _DUTY_NEU 1470 //서보 중립 위치
#define _DUTY_MAX 2070 //서보 최대 위치

// Servo speed control
#define _SERVO_ANGLE 70.0 //서보 구동 각도
#define _SERVO_SPEED 200.0 //서보 속도

// Event periods
#define _INTERVAL_DIST 20 //거리 측정 INTERVAL
#define _INTERVAL_SERVO 20 //서보 구동 INTERVAL
#define _INTERVAL_SERIAL 100 //Serial 출력 INTERVAL

// PID parameters
#define _KP 1.5 //비례 제어 상수
#define _KD 95  //미분 제어 상수
#define _KI 0.01 //적분 제어 상수

//////////////////////
// global variables //
//////////////////////

// Servo instanceL
Servo myservo;

// Distance sensor
float dist_target = 255; //목표 위치
float dist_raw, dist_ema; //거리 측정을 위한 변수 선언

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //샘플링 주기를 위한 변수 선언
bool event_dist, event_servo, event_serial; //주기를 체크하기 위한 boolean타입 변수 선언

// Servo speed control
int duty_chg_per_interval; //서보 속도 제어를 위한 변수 선언
int duty_target, duty_curr; //서보 속도 제어를 위한 변수 선언

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; //PID제어를 위한 변수 선언


//sensor_Noise filter
////////////////////////////////////////////////////////////////////////////
float under_noise_filter(void){
  float currReading;
  float largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  //largestReading = 100 + 300.0 / (b - a) * (largestReading - a);
  return largestReading;
}

float filtered_ir_distance(void){
  float currReading;
  float lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
////////////////////////////////////////////////////////////////////////////


void setup() {
  // initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo

  // initialize global variables
  iterm = 0; //iterm initiailze
  dterm = 0; //dterm initialize
  pterm = 0; //pterm initialize
  
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); 
  duty_curr = _DUTY_NEU;
  
  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * 
  (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
} 

void loop() { 
  /////////////////////
  // Event generator //
  ///////////////////// 

  unsigned long time_curr = millis(); //프로그램 구동 시간 
  //거리 측정 주기 체크 
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }

  //서보 구동 주기 체크
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }

  //시리얼 출력 주기 체크 
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }
  
  //거리 측정 함수 
  if(event_dist){
    event_dist = false;
    dist_ema = ir_distance_sequence();
    dist_raw2 = ir_distance();

    error_curr = _DIST_TARGET - dist_ema ;
    pterm = _KP * error_curr;
    iterm += _KI * error_curr;
    dterm = _KD * ( error_curr - error_prev );
    control = pterm + iterm + dterm;
    
    duty_target = _DUTY_NEU - control * ((control>0)?(_DUTY_MAX - _DUTY_NEU):(_DUTY_NEU - _DUTY_MIN))/(_DUTY_MAX - _DUTY_MIN);
;
    
    if(duty_target < _DIST_MIN) duty_target = _DUTY_MIN; 
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    error_prev = error_curr;
  }
  
  //서보 구동 함수 (dcpi적용을 통해 서보 속도 제어)
  if(event_servo){
    event_servo = false;
    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else{
      duty_curr -=duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    myservo.writeMicroseconds(duty_curr);
   }
  
  //시리얼 출력 함수
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(_DIST_TARGET);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm, -1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}
//적외선 거리 측정 함수
float ir_distance(void){
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return value;
}
//구간별 보정을 위한 코드
float ir_distance_sequence(void){
  float value = filtered_ir_distance();
  int s = 0, e = SEQ_SIZE - 1, m;
  // binary search
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]){
         e = m - 1;
    }
    else if(value > x[m+1]){
      s = m + 1;
    }
    else{
      break;
    }
  }
  if(s > e){
    if(value > 10000.0 || value < -10000.0) real_value = _DIST_TARGET;
    else if(s == 0) real_value = _DIST_MIN; 
    else real_value = _DIST_MAX;
  }
  // calculate real values
    real_value = (y[m+1] - y[m]) / (x[m+1] - x[m] ) * (value - x[m]) + y[m];
  return real_value;
}
