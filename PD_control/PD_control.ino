#include <Servo.h> 

/////////////////////////////
// Configurable parameters //
/////////////////////////////
#define SEQ_SIZE 8
float dist_raw2, raw_dist, real_value;
// sensor values
float x[8] = {70.0, 120.0, 165.0, 210.0, 255.0 , 270.0 , 290.0, 300.0};
  
// real values
float y[8] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0};

////////////////////////////
#define DELAY_MICROS  1500
#define EMA_ALPHA 0.35
float filtered_dist;
float ema_dist = 0;
float samples_num = 3;
///////////to filter////////


// Arduino pin assignment
#define PIN_LED 9   
#define PIN_SERVO 10 
#define PIN_IR A0  

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410 

// Distance sensor
#define _DIST_ALPHA 0.3  

// Servo range
#define _DUTY_MIN 1070
#define _DUTY_NEU 1470
#define _DUTY_MAX 1770
// Servo speed control
#define _SERVO_ANGLE 70.0 
#define _SERVO_SPEED 150.0

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 20

// PID parameters
#define _KP 2
#define _KD 75.0  //1.8,47
//#define a 70
//#define b 245

//////////////////////
// global variables //
//////////////////////

// Servo instanceL
Servo myservo;

// Distance sensor
float dist_target; 
float dist_raw, dist_ema; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr; 

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

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


void setup() {
  // put your setup code here, to run once:
  // initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins

  // initialize global variables

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_MAX);
  
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
    dist_ema = ir_distance_sequence();
    dist_raw2 = ir_distance();

    error_curr = _DIST_TARGET - dist_ema ;
    pterm = _KP * error_curr;
    iterm = 0;
    dterm = _KD * ( error_curr - error_prev );
    control = pterm + iterm + dterm;
    
    duty_target = _DUTY_NEU - control * ((control>0)?(_DUTY_MAX - _DUTY_NEU):(_DUTY_NEU - _DUTY_MIN))/(_DUTY_MAX - _DUTY_MIN);
;
    
    if(duty_target < _DIST_MIN) duty_target = _DUTY_MIN; 
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    error_prev = error_curr;
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
    
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,  510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    /*
    Serial.print(",error_curr:");
    Serial.print(error_curr);
    Serial.print("error_prev:,");
    Serial.println(error_prev);  
    */
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.print(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    
    Serial.print("No_filtered:");
    Serial.println(ir_distance());
  }
}

float ir_distance(void){
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return value;
}
/*
float ir_distance_filtered(void){
  raw_dist =  ir_distance();
  dist_raw = 100 + 300.0 / (b - a) * (raw_dist - a);
  return _DIST_ALPHA*dist_raw+(1-_DIST_ALPHA)*dist_ema;
}
*/


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

// examples for sequence calibration
