#include <Wire.h>
#include <LCDi2cR.h>
#include <XBee.h>
#define TMP102 0x48

//LCD
LCDi2cR lcd = LCDi2cR(4, 20, 0x63, 0);
uint8_t rows = 4;
uint8_t cols = 20;

//Xbee send
XBee xbee               = XBee();
uint8_t payload[8] = {0};
int Xbee = 0;
// SH + SL Address of receiving XBee                             // Remote XBee    
 XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4098DA06); // address of the receiver XBee
   
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

//Xbee receive
XBeeResponse response   = XBeeResponse();
ZBRxResponse        rx  = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

//variables for xbee received
int  angle = 0;
int mgauss=0;
int humidity=0;
float altitude = 0;
short temperature085=0;


void setup() {
  Wire.begin();
  Serial.begin(9600);

//xbee
  Serial2.begin(9600);
  xbee.setSerial(Serial2);

  //lcd
  lcd.init(); 
 

}

void loop() {
 int keyInput;
  lcd.println("Enter key");
  //delay(50);

  keyInput = lcd.keypad();
  lcd.print((keyInput-48), DEC);
  //delay(50);
  lcd.clear();

  
  sendxbee();


   Xbee = keyInput-48;
   payload[4] = Xbee >> 24 & 0xff;
   payload[5] = Xbee >> 16 & 0xff;
   payload[6] = Xbee >> 8 & 0xff;
   payload[7] = Xbee & 0xff; 
   
   xbee.send(zbTx);
   receivexbee();
}

void sendxbee() {

//xbee transmit
  //Temp102
      getTemp102();

}

void receivexbee() {
//xbee receive
  xbee.readPacket();

   if (xbee.getResponse().isAvailable())
    {
      // got something
      //Serial.print(rx.getDataLength(),DEC);
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) 
      {
        // got a zb rx packet
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
     //   if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED)
      
        angle = rx.getData(0) << 24 | rx.getData(1) << 16 | rx.getData(2) << 8 | rx.getData(3);
        Serial.print("Angle360 = ");
       Serial.println(angle);

       //LCD
       lcd.clear();
     lcd.print("Angle360 = ");
       lcd.print(angle);
       
      

       mgauss = rx.getData(4) << 24 | rx.getData(5) << 16 | rx.getData(6) << 8 | rx.getData(7);
       Serial.print("Mgauss= ");
       Serial.println(mgauss);

       //LCD
        //lcd.clear();
     lcd.print("Mgauss = ");
       lcd.print(mgauss);
       //delay(10);
       //lcd.clear();

        temperature085 = rx.getData(8) << 8 | rx.getData(9);
        Serial.print("Temperature085= ");
        Serial.println(temperature085*0.1);

        //LCD
          //lcd.clear();
     lcd.print("Temp085 = ");
       lcd.print(temperature085*0.1);
      // delay(10);
      // lcd.clear();

       altitude = rx.getData(10) << 24 | rx.getData(11) << 16 | rx.getData(12) << 8 | rx.getData(13);
       Serial.print("Altitude= ");
       Serial.println(altitude);

       //LCD
      
       //lcd.clear();
       lcd.print("Altitude = ");
       lcd.print(altitude);
      // delay(10);
      // lcd.clear();
       
       humidity = rx.getData(14) << 24 | rx.getData(15) << 16 | rx.getData(16) << 8 | rx.getData(17);
       Serial.print("Humidity= ");
       Serial.println(humidity);

       //LCD
       // lcd.clear();
       lcd.print("Humidity = ");
       lcd.print(humidity);
       delay(100);
       lcd.clear();

      }
    }
}
        
           
void getTemp102 (){
  byte firstByte, secondByte;
  int value;
  float convertedTemp;

  Wire.beginTransmission(TMP102);
  Wire.write((byte)0x00);
  Wire.endTransmission();
  Wire.requestFrom(TMP102, 2);

  if(Wire.available() >=2){
    firstByte = (Wire.read());
    secondByte = (Wire.read());
  }
  value = ((firstByte) << 4);
  value |= ((secondByte) >>4);
  convertedTemp = value/16;
 
//Xbee send
   Xbee = convertedTemp;
   payload[0] = Xbee >> 24 & 0xff;
   payload[1] = Xbee >> 16 & 0xff;
   payload[2] = Xbee >> 8 & 0xff;
   payload[3] = Xbee & 0xff;

}
