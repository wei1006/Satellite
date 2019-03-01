#include <LCDi2cR.h>
#include <Wire.h>
#include <math.h>
#include <XBee.h>
#include <SD.h>
#define BMP085 0x77
#define HMC5883L 0x1E

//SD card
File Satellite;
int SDcardCancel = 0;

//Xbee send
XBee xbee = XBee();
uint8_t payload[18] = {0};
int Xbee = 0;

// SH + SL Address of receiving XBee                             // Remote XBee    
 XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4098DA0D); // address of the receiver XBee
   
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

//Xbee receive
XBeeResponse response   = XBeeResponse();
ZBRxResponse        rx  = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

//variable for xbee reveive
float convertedTemp=0; //32 bits
int keypad=0;
int dataReceive;
int refresh;
int out;

//BMP085 values
const unsigned char OSS = 0;
int ac1, ac2, ac3;
unsigned int ac4, ac5, ac6;
int b1, b2;
int mb, mc, md;
long b5;
short temperature085; //16 bits
long pressure; //64 bits
const float p0 = 101325;
float altitude;//32 bits

//HIH-4030
const int outPin = A7; 
int value = 0;
int humidity; //32 bits

//Compass
int x, y, z;
int angle,pythagoras;//32 bits
int mgauss;//32 bits
unsigned int xp, yp, zp;

void setup() {
  //Arduino
  Wire.begin();
  Serial.begin(9600);
  
  //xBee
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  //SD card
  pinMode(53, OUTPUT);

  if (!SD.begin(53))
  {
    SDcardCancel = 1;
  }

  if (!(SD.exists("data.csv")))
  {
    Satellite = SD.open("data.csv", FILE_WRITE);
    Satellite.close();
  }

  //HMC5883L
  Wire.beginTransmission(HMC5883L);
  Wire.write((byte)0x02);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  //BMP085
  bmp085Calibration();
}

void loop() {

   //Sensors will obtain readings and readings are stored in variables

  //Compass
  getCompass();

  //BMP085
  temperature085 = getTemp085(bmp085ReadUT());
  pressure = getPressure(bmp085ReadUP());

  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));

  //HIH-4030
  value = analogRead(outPin); // read the voltage on the out pin
  humidity = (((( value /204.8)-0.958)/3) * 100); 
  
 //SD card saving data
 if (SDcardCancel == 0)
          {
            saveData();
          }

 //Xbee sends data from satellite to ground station first
 
 dataReceive = 0; //if any data is received from ground station
 refresh = 0;

 while ((dataReceive == 0) && (refresh < 10)) 
 //As long as data is not received from ground station, satellite will keep sending data over
 {
  //Xbee for compass
   Xbee = angle+180;
   payload[0] = Xbee >> 24 & 0xff;
   payload[1] = Xbee >> 16 & 0xff;
   payload[2] = Xbee >> 8 & 0xff;
   payload[3] = Xbee & 0xff;
   
    Xbee = mgauss;
   payload[4] = Xbee >> 24 & 0xff;
   payload[5] = Xbee >> 16 & 0xff;
   payload[6] = Xbee >> 8 & 0xff;
   payload[7] = Xbee & 0xff;

   Xbee = temperature085;
   payload[8] = Xbee >> 8 & 0xff;
   payload[9] = Xbee & 0xff;

   Xbee = altitude;
   payload[10] = Xbee >> 24 & 0xff;
   payload[11] = Xbee >> 16 & 0xff;
   payload[12] = Xbee >> 8 & 0xff;
   payload[13] = Xbee & 0xff;

   Xbee = humidity;
   payload[14] = Xbee >> 24 & 0xff;
   payload[15] = Xbee >> 16 & 0xff;
   payload[16] = Xbee >> 8 & 0xff;
   payload[17] = Xbee & 0xff;
   
   xbee.send(zbTx);

   checkData(); //function to check if any data is received from ground station
   refresh++;
 }
 
//function for xBee receiving data from ground station
if (refresh !=10)
{
  receiveData();
}
  
}

//function for xBee receiving data from ground station
void receiveData () 
{
  out = 0; //to determine if keypad is 0

  while (out == 0) //if keypad is 0 data will be received if not no data is received
  {
  xbee.readPacket();

    if (xbee.getResponse().isAvailable())
    {
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) 
      {
        xbee.getResponse().getZBRxResponse(rx);
        
        convertedTemp  = rx.getData(0) << 24 | rx.getData(1) << 16 | rx.getData(2) << 8 | rx.getData(3);

        keypad  = rx.getData(4) << 24 | rx.getData(5) << 16 | rx.getData(6) << 8 | rx.getData(7);
        interpretData(keypad);//to determine if keypad is 0
          
      }
     }
    }
}

//Set dataReceive to 1 if remote XBee has receive the data sent
void checkData()
{
  if (xbee.readPacket(1000))
  {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      if (txStatus.getDeliveryStatus() == SUCCESS)  //check if data is received by this xbee
      {
           
        dataReceive = 1;
      } 
    }      
  } 
}

//function to determine if keypad is 0
void interpretData(int keypad)
{
  if (keypad == 0)
  {
    out = 1; //if keypad is 0 xbee stops receiving data from ground station
  }
}

void getCompass () {
  
  Wire.beginTransmission(HMC5883L);
  Wire.write((byte)0x03);
  Wire.endTransmission();
  
  Wire.requestFrom(HMC5883L,6);
  
  //Reading data from axes
  if( 6 <= Wire.available() )
  {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  //Getting the angle
  angle = atan2(-y, x) * (180 / M_PI);
  xp = sq(x);
  yp = sq(y);
  zp = sq(z);
  pythagoras = sqrt(xp + yp);
  mgauss = 2*(pythagoras);
//Serial.println(mgauss);
  
}

void bmp085Calibration(){
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

short getTemp085(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return((b5 + 8) >> 4);
}

long getPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));

  if (b7 < 0x80000000){
    p = (b7 << 1)/b4;
  }
  else{
    p = (b7 / b4) << 1;
  }
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

char bmp085Read(unsigned char address){
  unsigned char data;

  Wire.beginTransmission(BMP085);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085, 1);
  while(!Wire.available());

  return Wire.read();
}

int bmp085ReadInt(unsigned char address){
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb << 8 | lsb;
}

unsigned int bmp085ReadUT(){
  unsigned int ut;

  Wire.beginTransmission(BMP085);
  Wire.write((byte)0xF4);
  Wire.write((byte)0x2E);
  Wire.endTransmission();

  delay(5);

  ut = bmp085ReadInt(0xF6);
  return ut;
}

unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  Wire.beginTransmission(BMP085);
  Wire.write((byte)0xF4);
  Wire.write((byte)(0x34 + (OSS<<6)));
  Wire.endTransmission();
  
  delay(2 + (3<<OSS));
  
  Wire.beginTransmission(BMP085);
  Wire.write((byte)0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085, 3);
  
  while(Wire.available() < 3);
  msb  = Wire.read();
  lsb  = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

void saveData()
{
  Satellite = SD.open("data.csv", FILE_WRITE);
  
  if (Satellite)
  {
    save();
    
    Satellite.close();
  } 
}

//Save data to SD Card
void save()
{
  String dataString = "";

  dataString = dataString + String(angle+180);
  dataString = dataString + ",";
  dataString = dataString + String(mgauss);
  dataString = dataString + ",";
  dataString = dataString + String(temperature085);
  dataString = dataString + ",";
  dataString = dataString + String(altitude);
  dataString = dataString + ",";
  dataString = dataString + String(humidity);

  Satellite.println(dataString);
}
