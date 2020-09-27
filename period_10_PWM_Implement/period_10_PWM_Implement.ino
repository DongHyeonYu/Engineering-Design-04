#define led 7

void setup(){
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  int i;
  int period = set_period(10);
  for(i=1; i<=100; i+=1){
    set_duty(period, i);
  }
  for(i=99; i>=0; i-=1){
    set_duty(period, i);
  }
}

int set_period(int period){
  return period*1000;
}

void set_duty(int period, int duty){
  double led_on = period/100*duty;
  double led_off = period - led_on;
  Serial.println(led_on);
  Serial.println(led_off);
  digitalWrite(led, LOW);
  delayMicroseconds(led_on);
  digitalWrite(led, HIGH);
  delayMicroseconds(led_off);
}
