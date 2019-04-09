
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2,3); // RX, TX

void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);


  Serial.println("Enter AT Command");
    Serial.println("Set Serial Monitor to both NL & CR & baud rate to 9600");
      Serial.println("Enter AT and press Enter if your get 'OK' then continue else check the above setting");
        Serial.println("Enter 'AT+CIOBAUD=19200' if your get 'OK' You have sucessfully set the communication baud rate");
         Serial.println("If Completed Sucessfully continue to Upload Step2 Code");

  // Set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
}

void loop() 
{ delay(10);
  if (mySerial.available()) 
  {
    Serial.write(mySerial.read());
  }
  
  if (Serial.available()) 
  {
    mySerial.write(Serial.read()); //to connect with the ESP with the PC
  }
}
