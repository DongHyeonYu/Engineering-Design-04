#define PIN_LED 7
int count;
void setup() {
  Serial.begin(9600);
  Serial.println("HelloWorld!");
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED, 1);
  delay(1000);
}

void loop() {
  ledFlashing();
  ledFlashing();
  ledFlashing();  
  ledFlashing();
  ledFlashing();
  while(1){
  digitalWrite(PIN_LED,HIGH);
  }  
}

int ledFlashing(){
  digitalWrite(PIN_LED,HIGH);
  delay(100);
  digitalWrite(PIN_LED,LOW);
  delay(100);
  Serial.println(++count);
}
