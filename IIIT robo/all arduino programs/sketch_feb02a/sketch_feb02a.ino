void setup() {
  // put your setup code here, to run once:
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite (5,LOW);
digitalWrite (6,LOW);
delay(2000);
digitalWrite (5,LOW);
digitalWrite (6,HIGH);
delay(2000);
}
