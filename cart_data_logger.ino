/* 
  cart_data_logger.ino

  Code to read appropriate sensors and log data for Strawberry Yield Mapping system
  Reads: Load cells, custom IMU device, Arduino GPS (shield), Piksi GPS
  Sends data out through UART0 to data logger + Xbee
  
  Hardware connections:
  UART0 - Data logger + Xbee
  UART1 - Shield GPS
  UART2 - Custom IMU
  UART3 - Piksi GPS
  Connect Pin 2 to Pin 19 (RX1)
  Connect Pin 3 to Pin 18 (TX1)
  Analog 0-3 - Load Cells (post-amplification)
  
  Adapted from "Load_cell_read" and "GPS_read" projects by same 
  Uses modified version of TinyGPS++ library
  
  Author: Richie Rehal
  Contact: rsrehal@ucdavis.edu
  Date: 5-13-2015
  Bio-Automation Lab
  
*/

#include "TinyGPS++.h"    // include user modified library from local directory
#include <math.h>

#define TIME_INTERVAL 200    //100ms intervals = 10 samples per second

// baud rates for communication
static const uint32_t COMBaud = 115200;  // data logger + Xbee
static const uint32_t GPSBaud = 4800;    // Shield GPS
static const uint32_t IMUBaud = 115200;    // custom IMU
static const uint32_t PiksiBaud = 115200;  // Piksi GPS

static const char delim = ',';  // delimeter character

// The TinyGPS++ objects
TinyGPSPlus shield_gps;
TinyGPSPlus piksi;

// variable for storing UTM conversion output
double utm[3] = {0}; // [0] = x (UTM easting in meters), 
                    // [1] = y (UTM northing in meters, 
                    // [2] = zone (UTM longitudinal zone)

int buttonPin = 20;        // corresponds to ISR.3 on Arduino Mega
int captureFlag = 0;

int analogPin[4] = {0, 1, 2, 3};
int loadCellVal[4] = {0};
float loadCellVoltage[5] = {0};   // last value is sum total

unsigned long currentTime = 0;
unsigned long prevTime = 0;
unsigned long timeStamp[4] = {0};  // used to store Arduino clock time stamps for data readings
                                   // order: load cells, IMU, shield GPS, Piksi GPS (time stamps)

//float accel[3] = {0};
//float gyro[3] = {0};
//float compass[3] = {0};
//float euler[3] = {0};              // order: roll, pitch, yaw
//float quatern[4] = {0};
float IMUvalues[16] = {0};        // order: accel x3, gyro x3, compass x2, euler (roll, pitch, yaw), 
                                  // quaternion x4
String IMUinput = "";

// constants for data output precision
static const int loadCellPrecision = 4;
static const int IMUPrecision = 4;
static const int GPSPrecision = 6;

//

void timedOutput()
{      
      while((millis() - prevTime) < TIME_INTERVAL)
      {
        //wait
      }  
}//timedOutput()

// convert longitude + latitude to UTM
// Originally from wgs2utm.m by Alexandre Schimel, MetOcean Solutions Ltd
// adapted from C++ code written by Raffi Zack
void wgs2utm(double latitude, double longitude)
{    
  //converting decimal degrees to UTM x,y
  double latitude_rad = latitude * M_PI/180; // latitude in rad
  double longitude_rad = longitude * M_PI/180; // longitude in rad
  
  //values
  double a = 6378137; // semi-major axis
  double b = 6356752.314245; // semi-minor axis
  double e = sqrt(1-pow((b/a),2)); // eccentricity
  double longitude_decimal0 = floor(longitude/6)*6+3; // ref longitude, deg
  double longitude_rad0 = longitude_decimal0 * M_PI/180; // ref longitude, rad
  double k0 = 0.9996; // central meridian scale
  double FE = 500000; // false easting
  double FN;
  if (latitude < 0) {
    FN = 10000000;
  }
  else {
    FN = 0; // false northing 
  }
  
  //equations
  double eps = pow(e,2)/(1-pow(e,2)); // e prime square
  double N = a/sqrt(1-pow(e,2)*pow(sin(latitude_rad),2)); // earth radius of curvature perpend. to meridian plane
  double T = pow(tan(latitude_rad),2);
  double C = ((pow(e,2))/(1-pow(e,2)))*pow(cos(latitude_rad),2);
  double A = (longitude_rad-longitude_rad0)*cos(latitude_rad);
  double M = a*((1-pow(e,2)/4 - 3*pow(e,4)/64 - 5*pow(e,6)/256)*latitude_rad - (3*pow(e,2)/8 + 3*pow(e,4)/32 + 45*pow(e,6)/1024)*sin(2*latitude_rad) + (15*pow(e,4)/256 + 45*pow(e,6)/1024)*sin(4*latitude_rad) - (35*pow(e,6)/3072)*sin(6*latitude_rad)); // true distance along the central meridian from equator to latitude
  double x = FE + k0*N*(A + (1-T+C)*pow(A,3)/6 + (5-18*T+pow(T,2)+72*C-58*eps)*pow(A,5)/120); // easting
  double y = FN + k0*M + k0*N*tan(latitude_rad)*(pow(A,2)/2 + (5-T+9*C+4*pow(C,2))*pow(A,4)/24 + (61-58*T+pow(T,2)+600*C-330*eps)*pow(A,6)/720); // northing

  utm[0] = x;
  utm[1] = y;
  
  //Serial.println("I got here to UTM!!!");
  
  
}//wgs2utm()

void capture()
{
   // ISR
   // when button is pressed, capture and transmit data
   captureFlag = 1;
   
}//capture()

void readLoadCells()
{
    float totalVoltage = 0;
/*
    Serial.println();
    Serial.println();
    Serial.println("Printing load cell readings...");
    Serial.println();
*/

    timeStamp[0] = millis();

    for(int i = 0; i < 4; i++)
    {
      loadCellVal[i] = analogRead(analogPin[i]);        

/*
      // output data
      Serial.print("Load cell ");
      Serial.print(i+1);
      Serial.print(" = , ");
      //Serial.print(" , value = ");
      //Serial.print(loadCellVal[i]);
*/ 
      loadCellVoltage[i] = ((float)loadCellVal[i] / 1024) * 5;
      //Serial.print(" , \t "); 

      totalVoltage += loadCellVoltage[i]; 
/*      
      Serial.print(loadCellVoltage[i], 3);
	//Serial.println(" V");
      Serial.println();  
*/
    }//for i
    
    loadCellVoltage[4] = totalVoltage;
    
/*    
    Serial.print("Time (ms) = ,,,, \t\t\t ");
    Serial.println(timeStamp[0]);
*/    

}//readLoadCells()


// reads from IMU device through Serial2
void readMotionData(void)
{
  int i = 0;
  char inChar = 0;
  
  //first send byte indicating ready to recieve
  Serial2.print('r');
  timeStamp[1] = millis();
  
  //wait
  //delay(50);
  
  while(i < 16)
  {
    if(Serial2.available())
    {  
      inChar = Serial2.read();
      //delay(10);
      //Serial.print(inChar);
      
      if(inChar != '\n')
      {
        IMUinput += inChar;
      }else{
        //valid float found
        IMUvalues[i++] = IMUinput.toFloat();
        //Serial.println(IMUvalues[i-1]);
        IMUinput = "";
      }
    }
  }//while all data not received
     
  Serial2.flush();
  i = 0;

/*   
   // output data to Serial
   Serial.print("Accel X =, ");
   Serial.println(IMUvalues[0], 6);
   Serial.print("Accel Y =, ");
   Serial.println(IMUvalues[1], 6);
   Serial.print("Accel Z =, ");
   Serial.println(IMUvalues[2], 6);
   Serial.print("Roll =, ");
   Serial.println(IMUvalues[9], 6);
   Serial.print("Pitch =, ");
   Serial.println(IMUvalues[10], 6);
   Serial.print("Yaw =, ");
   Serial.println(IMUvalues[11], 6);
   Serial.print("Quaternion 1 =, ");
   Serial.println(IMUvalues[12], 6);
   Serial.print("Quaternion 2 =, ");
   Serial.println(IMUvalues[13], 6);
   Serial.print("Quaternion 3 =, ");
   Serial.println(IMUvalues[14], 6);
   Serial.print("Quaternion 4 =, ");
   Serial.println(IMUvalues[15], 6);
   Serial.print("Time (ms) =,, ");
   Serial.println(timeStamp[1]);
   Serial.println();
*/   
  
}//readMotionData()

void readGPSData(void)
{  
  // get and decode GPS NMEA sentence
  // from shield GPS
  
/*  
  Now also gives GPS Fix Status/Quality
        Fix quality:           0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
			       4 = Real Time Kinematic
			       5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
			       7 = Manual input mode
			       8 = Simulation mode
*/  

  int breakFlag = 0;
  //Serial1.flush();
  //delay(500);
  while (Serial1.available() > 0 && breakFlag == 0)
  {
    if (shield_gps.encode(Serial1.read()))
    {
      timeStamp[2] = millis();
      breakFlag = 1;
      //displayInfo();
      
      //Serial.println(gps.altitude.meters());    //double
      //Serial.println(gps.satellites.value());    //u32 (int)
/*      
      wgs2utm(shield_gps.location.lat(), shield_gps.location.lng());
      Serial.print("UTM Easting = ");
      Serial.println(utm[0]);
      Serial.print("UTM Northing = ");
      Serial.println(utm[1]);
*/      
    }//if
    
  }//while

  if (shield_gps.charsProcessed() < 10)
  {
    //Serial.println(F("No Shield GPS detected: check wiring."));
    timeStamp[2] = 0;
    //while(true);
  }
  
  // get and decode GPS NMEA sentence
  // from Piksi GPS
  while (Serial3.available() > 0 && breakFlag == 1)
  {
    if (piksi.encode(Serial3.read()))
    {
      timeStamp[3] = millis();
      breakFlag == 0;
      //displayInfo();
      
      //Serial.println(gps.altitude.meters());    //double
      //Serial.println(gps.satellites.value());    //u32 (int)
/*      
      wgs2utm(piksi.location.lat(), piksi.location.lng());
      Serial.print("UTM Easting = ");
      Serial.println(utm[0]);
      Serial.print("UTM Northing = ");
      Serial.println(utm[1]);
*/      
    }//if
    
  }//while

  if (piksi.charsProcessed() < 10)
  {
    //Serial.println(F("No Piksi GPS detected: check wiring."));
    timeStamp[3] = 0;
    //while(true);
  }
  
  
}//readGPSdata()

void printHeader(void)
{
  // print header for data output
  // defines order of data output
  Serial.print(F("Load Cell Arduino Timestamp"));
  Serial.print(delim);
  Serial.print(F("Load Cell 1 (V)"));
  Serial.print(delim);
  Serial.print(F("Load Cell 2 (V)"));
  Serial.print(delim);
  Serial.print(F("Load Cell 3 (V)"));
  Serial.print(delim);
  Serial.print(F("Load Cell 4 (V)"));
  Serial.print(delim);
  Serial.print(F("Load Cell TOTAL (V)"));
  Serial.print(delim);
  
  Serial.print(F("IMU Arduino Timestamp"));
  Serial.print(delim);
  Serial.print(F("Accel X"));
  Serial.print(delim);
  Serial.print(F("Accel Y"));
  Serial.print(delim);
  Serial.print(F("Accel Z"));
  Serial.print(delim);
  Serial.print(F("Gyro X"));
  Serial.print(delim);
  Serial.print(F("Gyro Y"));
  Serial.print(delim);
  Serial.print(F("Gyro Z"));
  Serial.print(delim);
  Serial.print(F("Compass X"));
  Serial.print(delim);
  Serial.print(F("Compass Y"));
  Serial.print(delim);
  Serial.print(F("Compass Z"));
  Serial.print(delim);
  Serial.print(F("Roll (deg)"));
  Serial.print(delim);
  Serial.print(F("Pitch (deg)"));
  Serial.print(delim);
  Serial.print(F("Yaw (deg)"));
  Serial.print(delim);  
  Serial.print(F("Quaternion 1"));
  Serial.print(delim);  
  Serial.print(F("Quaternion 2"));
  Serial.print(delim);  
  Serial.print(F("Quaternion 3"));
  Serial.print(delim);  
  Serial.print(F("Quaternion 4"));
  Serial.print(delim);
    
  Serial.print(F("Shield GPS Arduino Timestamp"));
  Serial.print(delim);
  Serial.print(F("GPS Time (UTC)"));
  Serial.print(delim);
  Serial.print(F("Latitude (deg)"));
  Serial.print(delim);
  Serial.print(F("Longitude (deg)"));
  Serial.print(delim);
  Serial.print(F("UTM Easting (m)"));
  Serial.print(delim);
  Serial.print(F("UTM Northing (m)"));
  Serial.print(delim);
  Serial.print(F("GPS Altitude (m)"));
  Serial.print(delim);
  Serial.print(F("# GPS Satellites"));
  Serial.print(delim);
  Serial.print(F("GPS Fix Quality"));
  Serial.print(delim);  
  
  Serial.print(F("Piksi GPS Arduino Timestamp"));
  Serial.print(delim);
  Serial.print(F("GPS Time (UTC)"));
  Serial.print(delim);
  Serial.print(F("Latitude (deg)"));
  Serial.print(delim);
  Serial.print(F("Longitude (deg)"));
  Serial.print(delim);
  Serial.print(F("UTM Easting (m)"));
  Serial.print(delim);
  Serial.print(F("UTM Northing (m)"));
  Serial.print(delim);
  Serial.print(F("GPS Altitude (m)"));
  Serial.print(delim);
  Serial.print(F("# GPS Satellites"));
  Serial.print(delim);
  Serial.print(F("GPS Fix Quality"));
  Serial.print(delim);  
  
  Serial.println();
  
}//printHeader()

void dataOutput(void)
{
  // print data for logging
  int i = 0;
  
  // Load cells
  Serial.print(timeStamp[0]);
  Serial.print(delim);
  for(i = 0; i < 5; i++)
  {
    Serial.print(loadCellVoltage[i], loadCellPrecision);
    Serial.print(delim);
  }//for i

  // custom IMU  
  Serial.print(timeStamp[1]);
  Serial.print(delim);
  for(i = 0; i < 16; i++)
  {
    Serial.print(IMUvalues[i], IMUPrecision);
    Serial.print(delim);
  }//for
    
    
  // shield GPS
  Serial.print(timeStamp[2]);
  Serial.print(delim);
  
  if (shield_gps.time.isValid())
  {
    if (shield_gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(shield_gps.time.hour());
    Serial.print(F(":"));
    if (shield_gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(shield_gps.time.minute());
    Serial.print(F(":"));
    if (shield_gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(shield_gps.time.second());
    Serial.print(F("."));
    if (shield_gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(shield_gps.time.centisecond());
    
    Serial.print(delim);
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.print(delim);
  }
  
  if (shield_gps.location.isValid())
  {
    Serial.print(shield_gps.location.lat(), GPSPrecision);
    Serial.print(delim);
    Serial.print(shield_gps.location.lng(), GPSPrecision);
    Serial.print(delim);
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.print(delim);
    Serial.print(delim);
  }

  wgs2utm(shield_gps.location.lat(), shield_gps.location.lng());
  Serial.print(utm[0]);
  Serial.print(delim);
  Serial.print(utm[1]);
  Serial.print(delim);
  
  Serial.print(shield_gps.altitude.meters());
  Serial.print(delim);
  Serial.print(shield_gps.satellites.value());
  Serial.print(delim);
  Serial.print(shield_gps.fixQuality.value());
  Serial.print(delim);


  // Piksi GPS
  Serial.print(timeStamp[3]);
  Serial.print(delim);
  
  if (piksi.time.isValid())
  {
    if (piksi.time.hour() < 10) Serial.print(F("0"));
    Serial.print(piksi.time.hour());
    Serial.print(F(":"));
    if (piksi.time.minute() < 10) Serial.print(F("0"));
    Serial.print(piksi.time.minute());
    Serial.print(F(":"));
    if (piksi.time.second() < 10) Serial.print(F("0"));
    Serial.print(piksi.time.second());
    Serial.print(F("."));
    if (piksi.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(piksi.time.centisecond());
    
    Serial.print(delim);
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.print(delim);
  }
  
  if (shield_gps.location.isValid())
  {
    Serial.print(piksi.location.lat(), GPSPrecision);
    Serial.print(delim);
    Serial.print(piksi.location.lng(), GPSPrecision);
    Serial.print(delim);
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.print(delim);
    Serial.print(delim);
  }

  wgs2utm(piksi.location.lat(), piksi.location.lng());
  Serial.print(utm[0]);
  Serial.print(delim);
  Serial.print(utm[1]);
  Serial.print(delim);
  
  Serial.print(piksi.altitude.meters());
  Serial.print(delim);
  Serial.print(piksi.satellites.value());
  Serial.print(delim);
  Serial.print(piksi.fixQuality.value());
  Serial.print(delim);
  

  
  // prepare for next row of data
  Serial.println();
  
  
}//dataOutput()

void setup()
{
  
  //start main serial
  Serial.begin(COMBaud);

  // Shield GPS  
  Serial1.begin(GPSBaud);
  
  // custom IMU
  Serial2.begin(IMUBaud);
  
  // Piksi GPS
  Serial3.begin(PiksiBaud);
  
  //setup pins
  pinMode(buttonPin, INPUT);
  attachInterrupt(3, capture, LOW);  //interrupt on button press
                                    //button = active low                                    
  
  Serial.flush();
  delay(1000);            // delay to allow the Logomatic data logger to start up properly
  printHeader();
  
  
}//setup()

void loop()
{
  //char RXbyte = 0;

  if(true)
  //if(captureFlag == 1)
  {
    
    //Serial.println("Got to capture flag == 1 !!");
    
    readLoadCells();   
    //Serial.println("Read load cells successfully");
    readMotionData();
    //Serial.println("Read motion data successfully");
    readGPSData();
    
    dataOutput();
    
    captureFlag = 0;
  }



  // Timed output sequence to sync data output rate
  currentTime = millis();  
  timedOutput();  
  prevTime = millis();


  
}//loop()
