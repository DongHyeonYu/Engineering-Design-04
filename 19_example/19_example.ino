#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0

#define PIN_SERVO 10

#define _DUTY_MIN 1220 //down
#define _DUTY_NEU 1476 //neutral
#define _DUTY_MAX 1732 //up


#define _SERVO_SPEED 700 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

 
Servo servo;
int a, b; // unit: mm

unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;

unsigned long last_sampling_time_dist, last_sampling_time_serial, last_sampling_time_servo; // unit: ms
bool event_dist, event_serial, event_servo;
float raw_dist, dist_cali;

#define _INTERVAL_DIST 20   // USS interval (unit: ms)
#define _INTERVAL_SERIAL 20// serial interval (unit: ms)
#define _INTERVAL_SERVO 10

#define DELAY_MICROS  1500
#define EMA_ALPHA 0.35
float filtered_dist;
float ema_dist = 0;
float samples_num = 3;

void setup() {
  servo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
  servo.writeMicroseconds(_DUTY_NEU);
  //while(1){}
  
// initialize serial port
  Serial.begin(57600);

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  a = 70;
  b = 335;
}

float under_noise_filter(void){
  float currReading;
  float largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  float currReading;
  float lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(event_dist) {
    event_dist = false;
    raw_dist = filtered_ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
    
  }
  if(event_serial) {
    event_serial = false;    
    // output the read value to the serial port
    Serial.print("min:0,max:500,dist_cali:");
    Serial.print(dist_cali);
    Serial.print(", raw_dist:");
    Serial.println(raw_dist);
  }
  if(dist_cali > 255.0) {
    duty_target = _DUTY_MIN;
    //Serial.println("BBBB");
  }
  else{
    duty_target = _DUTY_MAX;
    //Serial.println("CCCC");
  }
  
  if(event_servo){
  if(duty_target > duty_curr) { 
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else{
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  servo.writeMicroseconds(duty_curr);
  }
  /*
  if(raw_dist < 255.0) {
    servo.writeMicroseconds(_DUTY_MIN);
    delay(100);
  }
  else if(raw_dist == 255.0){
    servo.writeMicroseconds(_DUTY_NEU);
    delay(100);
  }
  else {
    servo.writeMicroseconds(_DUTY_MAX);
    delay(100);
  }
  */
  delay(20);
}
