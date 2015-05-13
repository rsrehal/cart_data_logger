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
  
  Adapted from "Load_cell_read" and "GPS_read" projects by same author
  
  Author: Richie Rehal
  Contact: rsrehal@ucdavis.edu
  Date: 5-13-2015
  Bio-Automation Lab
  
*/

#include <TinyGPS++.h>
#include <math.h>

#define TIME_INTERVAL 100    //100ms intervals = 10 samples per second

// baud rates for communication
static const uint32_t COMBaud = 115200;  // data logger + Xbee
static const uint32_t GPSBaud = 4800;    // Shield GPS
static const uint32_t IMUBaud = 115200;    // custom IMU
static const uint32_t PiksiBaud = 115200;  // Piksi GPS

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

float accel[3] = {0};
float gyro[3] = {0};
float compass[3] = {0};
float euler[3] = {0};
float quatern[4] = {0};
float IMUvalues[16] = {0};

String IMUinput = "";

void timedOutput()
{      
      while(millis() - prevTime < TIME_INTERVAL)
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
    for(int i = 0; i < 4; i++)
    {
      loadCellVal[i] = analogRead(analogPin[i]);        


      // output data
      Serial.print("Load cell ");
      Serial.print(i+1);
      Serial.print(" = , ");
      //Serial.print(" , value = ");
      //Serial.print(loadCellVal[i]);
 
      loadCellVoltage[i] = ((float)loadCellVal[i] / 1024) * 5;
      //Serial.print(" , \t "); 

      totalVoltage += loadCellVoltage[i]; 
      Serial.print(loadCellVoltage[i], 3);
	//Serial.println(" V");
      Serial.println();  

    }//for i
    
    loadCellVoltage[5] = totalVoltage;
    

}//readLoadCells()


// reads from IMU device through Serial2
void readMotionData(void)
{
  int i = 0;
  char inChar = 0;
  
  //first send byte indicating ready to recieve
  Serial2.print('r');
  currentTime = millis();
  
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
   Serial.println(currentTime);
   Serial.println();
      
  
}//readMotionData()

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
  
}//setup()

void loop()
{
  char RXbyte = 0;


  if(captureFlag == 1)
  {
      readLoadCells();
      
      readMotionData();
      
      captureFlag = 0;
  }


/*
  // Timed output sequence to sync data output rate
  currentTime = millis();  
  timedOutput();  
  prevTime = millis();
*/

  
}//loop()
