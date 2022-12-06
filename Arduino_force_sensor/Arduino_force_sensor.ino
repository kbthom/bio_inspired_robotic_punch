// ****************** MUST INCLUDE LINES BELOW*****************************
// ****************** MUST INCLUDE LINES ABOVE*******1**********************
int forcePin = 14;
int reading = 0;
float newton_conversion = 0;
unsigned long timer = 0;
#define MATLABSerialPrint
char dataStr[100] = "";
char buff[5];

void setup() {
  // put your setup code here, to run once:
  pinMode(forcePin, INPUT);


  Serial.begin(115200); //begin serial comms
  delay(1000);
}

void loop() {
  dataStr[0] = 0;
  timer = millis() - timer;

  // put your main code here, to run repeatedly:
  reading = analogRead(forcePin) - 370;
  newton_conversion = (reading * (555/50) * (.0098));

//  Serial.println(newton_conversion);
  //Serial.println(newton_conversion);

//  //ltoa( timer/1000,buff,10);//vert long to charStr
//  strcat(dataStr, buff); //add it to the end
//  strcat( dataStr, ", "); //append the delimiter
//  Serial.println(dataStr);
//Serial.println(timer/1000);
//  dtostrf(newton_conversion, 3, 2,buff);  //5 is minimum width, 1 is precision; float value is copied onto buff
//  Serial.println(dataStr);
//  Serial.println(buff);
//  strcat( dataStr, buff); //append the converted float
//  strcat( dataStr, 0); //append the delimiter
//
//  Serial.println(dataStr);
//  delay(100);

  #ifdef MATLABSerialPrint
  Serial.print(millis() / 1000.0);   Serial.print(",");

  Serial.print(newton_conversion);
  Serial.println();  
  //delay(100);
  #endif

  
}
