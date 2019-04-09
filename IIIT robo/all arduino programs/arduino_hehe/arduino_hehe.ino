void setup() {
  // put your setup code here, to run once:
pinMode (12,OUTPUT); //fr red
pinMode (11,OUTPUT); //fl red
pinMode (7,OUTPUT); //bl green
pinMode (8,OUTPUT); //br green
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite (12,HIGH);
delay(1000);
digitalWrite(12,LOW);
delay(100);
digitalWrite (11,HIGH);
delay(100);
digitalWrite(11,LOW);
delay(100);
digitalWrite (7,HIGH);
delay(100);
digitalWrite(7,LOW);
delay(100);
digitalWrite (8,HIGH);
delay(100);
digitalWrite(8,LOW);
delay(100);
}
