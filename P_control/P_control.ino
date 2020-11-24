#include <Servo.h> //[3104] Servo header File include

/////////////////////////////
// Configurable parameters //
/////////////////////////////
#define SEQ_SIZE 8
float dist_raw2, raw_dist;
// sensor values
float x[8] = {70.0, 110.0, 150.0, 175.0, 195.0, 215.0, 245.0, 250.0};

// real values
float y[8] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0};


// Arduino pin assignment
#define PIN_LED 9 // [3110] 9번핀 LED 연결  
#define PIN_SERVO 10 // [3110] 10번핀 서보 연결
#define PIN_IR A0 //[3104] 적외선 거리센서 PIN - Analog0 정의 

// Framework setting
#define _DIST_TARGET 255 //[3104] 탁구공을 위치 시킬 목표 
#define _DIST_MIN 100 //[3117] 거리 최소값
#define _DIST_MAX 410 //[3117] 거리 최대값

// Distance sensor
#define _DIST_ALPHA 0.3  //[3099] EMA 필터링을 위한 alpha 값
               // [3108] 0~1 사이의 값

// Servo range
#define _DUTY_MIN 1143   //[3100] 최저 서보 위치 // up
#define _DUTY_NEU 1500    //[3100] 중립 서보 위치
#define _DUTY_MAX 1857  //[3100] 최대 서보 위치 //Down

// Servo speed control
#define _SERVO_ANGLE 50.0 //본인 서보의 실제 구동범위? 
#define _SERVO_SPEED 50.0

// Event periods
#define _INTERVAL_DIST 15 //[3099] 각 event 사이에 지정한 시간 간격
#define _INTERVAL_SERVO 10
#define _INTERVAL_SERIAL 20

// PID parameters
#define _KP 1.0

#define a 70
#define b 280

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //[1928] 측정된 값과 ema 필터를 적용한 값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
//[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial; 
//[3104] 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; // [3116] 주기 당 서보 duty값 변화량
int duty_target, duty_curr; //[1928] 목표 위치와 현재 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수

void setup() {
  // put your setup code here, to run once:
  // initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

  // initialize global variables

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

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }

  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO ){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }

  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }

  if(event_dist){
    event_dist = false;
    dist_ema = ir_distance_filtered();
    dist_raw2 = ir_distance();

    error_curr = dist_ema - _DIST_TARGET;
    pterm = error_curr;
    iterm = 0;
    dterm = 0;
    control = _KP* pterm + iterm + dterm;

    duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MAX-_DUTY_NEU):(_DUTY_NEU-_DUTY_MIN))/(_DUTY_MAX-_DUTY_MIN);
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MIN;
    else if(duty_target < _DUTY_MIN) duty_target = _DUTY_MAX; 
    
  }
  
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
  
      
  if(event_serial) {
    event_serial = false; 
    Serial.print("Min:0,Low:200,dist:");
    Serial.print(dist_ema);
    Serial.print("no_filtered:");
    Serial.print(dist_raw2);
    Serial.print("dcpi:"); 
    Serial.print(duty_chg_per_interval);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",High:310,Max:2000");
  }
}

float ir_distance(void){
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return value;
}
float ir_distance_filtered(void){
  raw_dist = ir_distance();
  dist_raw = 100 + 300.0 / (b - a) * (raw_dist - a);
  return _DIST_ALPHA*dist_raw+(1-_DIST_ALPHA)*dist_ema;
}


float ir_distance_sequence(void){
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
 
  int s = 0, e = SEQ_SIZE - 1, m;
  // binary search
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]){
         s = m + 1;
    }
    else if(value > x[m+1]){
      e = m - 1;
    }
    else{
      break;
    }
  }
  // out of sequence range: set value to neutral position
  // 센서 전원 Off 시, 오차값 발생으로 구간에서 벗어났을 시 등 
   if(s > e){
    if(value > 10000.0 || value < -10000.0) real_value = _DIST_TARGET;
    else if(s == 0) real_value = _DIST_MIN; 
    else real_value = _DIST_MAX;
  }


  // calculate real values
  real_value = (y[m+1] - y[m]) / (x[m+1] - x[m] ) * (value - x[m]) + y[m];
  return real_value;
}

// examples for sequence calibration

 
