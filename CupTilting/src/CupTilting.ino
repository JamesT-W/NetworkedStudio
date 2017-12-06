#include "MPU9150.h"
#include "Si1132.h"
#include "Si70xx.h"
#include "math.h"



//// Initialize application variables
#define RAD_TO_DEGREES 57.2957795131
#define DEG_TO_RADIANS 0.0174533
#define PI 3.1415926535
#define ACCEL_SCALE 2 // +/- 2g

//SERVER CONNECTION
TCPClient client;  //used to connect to the server
const char serverURL[] = "sccug-330-03.lancs.ac.uk"; //ip address
const int serverPort = 80;  //port

LEDStatus blinkYellow(RGB_COLOR_YELLOW, LED_PATTERN_SOLID, LED_SPEED_SLOW);

int PUBLISH_DELAY = 400; //adds a delay to make publishing less frequent
int SLEEP_DELAY = 30000; //adds a delay after publishing so that the following publishes print correctly (ms)
long PHOTON_SLEEP = 1800; // Seconds X2
int SENSORDELAY = 500;  //// 500; //3000; // milliseconds (runs x1)
//int EVENTSDELAY = 1000; //// milliseconds (runs x10)
//int OTAUPDDELAY = 7000; //// milliseconds (runs x1)
//int SLEEP_DELAY = 0;    //// 40 seconds (runs x1) - should get about 24 hours on 2100mAH, 0 to disable and use RELAX_DELAY instead
String SLEEP_DELAY_MIN = "15"; // seconds - easier to store as string then convert to int
String SLEEP_DELAY_STATUS = "OK"; // always OK to start with
int RELAX_DELAY = 5; // seconds (runs x1) - no power impact, just idle/relaxing

// Variables for the I2C scan
byte I2CERR, I2CADR;

//// ***************************************************************************
//// ***************************************************************************

int I2CEN = D2;
int ALGEN = D3;
int LED = D7;

int SOUND = A0;
double SOUNDV = 0; //// Volts Peak-to-Peak Level/Amplitude

int POWR1 = A1;
int POWR2 = A2;
int POWR3 = A3;
double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts

int SOILT = A4;
double SOILTV = 0; //// Celsius: temperature (C) = Vout*41.67-40 :: Temperature (F) = Vout*75.006-40

int SOILH = A5;
double SOILHV = 0; //// Volumetric Water Content (VWC): http://www.vegetronix.com/TechInfo/How-To-Measure-VWC.phtml

bool BMP180OK = false;
double BMP180Pressure = 0;    //// hPa
double BMP180Temperature = 0; //// Celsius
double BMP180Altitude = 0;    //// Meters

bool Si7020OK = false;
double Si7020Temperature = 0; //// Celsius
double Si7020Humidity = 0;    //// %Relative Humidity

bool Si1132OK = false;
double Si1132UVIndex = 0; //// UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //// Lux
double Si1132InfraRd = 0; //// Lux

MPU9150 mpu9150;
bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
double tm; //// Celsius

//float initialAccelX = 0;
//float initialAccelY = 0;
//float initialAccelZ = 0;
//float initialAccelXYZ = 0;
float initialTiltX = 0;
//float initialTiltY = 0;
int oldXTiltValue = 0;  //the xTiltValue stored at the end of the loop for the next loop
bool firstLoop = true;

//float newAccelX = 0;
//float newAccelY = 0;
//float newAccelZ = 0;
//float newAccelXYZ = 0;
float newTiltX = 0;
//float newTiltY = 0;

int soundState = 0; //initial measurement
int soundValue = 0; //second measurement to be compared
bool calibration = true;

//// ***************************************************************************

//// SYSTEM_MODE(SEMI_AUTOMATIC);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPinsMode()
{
    pinMode(I2CEN, OUTPUT);
    pinMode(ALGEN, OUTPUT);
    pinMode(LED, OUTPUT);

    pinMode(SOUND, INPUT);

    pinMode(POWR1, INPUT);
    pinMode(POWR2, INPUT);
    pinMode(POWR3, INPUT);

    pinMode(SOILT, INPUT);
    pinMode(SOILH, INPUT);
}

void setup()
{
    // opens serial over USB
    Serial.begin(9600);

    // Set I2C speed
    // 400Khz seems to work best with the Photon with the packaged I2C sensors
    Wire.setSpeed(CLOCK_SPEED_400KHZ);

    Wire.begin();  // Start up I2C, required for LSM303 communication

    // diables interrupts
    noInterrupts();

    // initialises the IO pins
    setPinsMode();

    // initialises MPU9150 inertial measure unit
    initialiseMPU9150();

    blinkYellow.setActive(true);
    connectVM();

/*
    int counter = 50;
    for (int i=0; i<counter; i++) { //get's an average for the initial values
      delay(100);
      readMPU9150();          //// reads compass, accelerometer and gyroscope data

      //initialAccelX += getAccelX(ax); initialAccelY += getAccelY(ay); initialAccelZ += getAccelZ(az);
      //initialAccelXYZ +=  getAccelXYZ(ax, ay, az);
      initialTiltX += getXTilt(ax,az);  //initialTiltY += getYTilt(ay,az);
    }
*/

    //initialAccelX = initialAccelX / 50; initialAccelY = initialAccelY / 50; initialAccelZ = initialAccelZ / 50;
    //initialAccelXYZ = initialAccelXYZ / 50;
    //initialTiltX = initialTiltX / counter; //initialTiltY = initialTiltY / 50;
    initialTiltX = 90;
    oldXTiltValue = 100;

}

//attempt to connect to VMserver, blink red if unable to
void connectVM(){
  interrupts();
  if(client.connect(serverURL, serverPort))
  {
    blinkYellow.setActive(false);
  }
  noInterrupts();
}

void initialiseMPU9150()
{
  ACCELOK = mpu9150.begin(mpu9150._addr_motion); // Initialize MPU9150

  if (ACCELOK)
  {
      // Clear the 'sleep' bit to start the sensor.
      mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);

      /// Set up compass
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x0F); //SelfTest
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode

      mpu9150.writeSensor(mpu9150._addr_motion, 0x24, 0x40); //Wait for Data at Slave0
      mpu9150.writeSensor(mpu9150._addr_motion, 0x25, 0x8C); //Set i2c address at slave0 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x26, 0x02); //Set where reading at slave 0 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x27, 0x88); //set offset at start reading and enable
      mpu9150.writeSensor(mpu9150._addr_motion, 0x28, 0x0C); //set i2c address at slv1 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x29, 0x0A); //Set where reading at slave 1 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x2A, 0x81); //Enable at set length to 1
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //overvride register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x67, 0x03); //set delay rate
      mpu9150.writeSensor(mpu9150._addr_motion, 0x01, 0x80);

      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x04); //set i2c slv4 delay
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x00); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x00); //clear usr setting
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x20); //enable master i2c mode
      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x13); //disable slv4
    }
    else
    {
      Serial.println("Unable to start MPU5150");
    }
}

void loop(void)
{
    //// powers up sensors
    digitalWrite(I2CEN, HIGH);
    digitalWrite(ALGEN, HIGH);

    delay(SENSORDELAY);     //// allows sensors time to warm up
    readMPU9150();          //// reads compass, accelerometer and gyroscope data
    interrupts();
    if(client.connected() != true)
    {
      blinkYellow.setActive(true);
      Serial.println("Connection to server lost");
      connectVM();
    }

    //Calibrate sound, measure ambient noise levels
    if(calibration) {
      soundState = measure();
      Serial.println("Calibrated!");
      calibration = false;
    }

    newTiltX = getXTilt(ax, az);

    float XTiltFraction = (newTiltX / initialTiltX) * 100; //turn 360 degrees into percentage (90 degrees (vertically upright) = 100%)
    int XTiltValue = round(XTiltFraction/1)*1;  //rounds the float to an int to remove the decimal places

    bool reversed = false;  //used if XTiltValue exceeds 100% (it should )
    String percentFull = "";

    //TiltValue is a percentage. 0% is horizontally empty in one direction, 100% is vertically upright,
    //200% is horizontally empty in the other direction, and >200% is upside down, therefore empty
    if (XTiltValue >= 95 && XTiltValue <= 105) {  //if nearly full, round up to full
      XTiltValue = 100;
    }
    else if (XTiltValue <= 5 || XTiltValue >= 395) {  //if nearly empty, round down to empty
      XTiltValue = 0;
    }
    else if (XTiltValue > 195 && XTiltValue < 400) {  //195-200% is the rounding threshold for the reverse direction, else cup is upside down
      XTiltValue = 0;
    }
    else if (XTiltValue > 105 && XTiltValue <= 195) { //if tilting in reverse direction, reverse values (when values go above 100%)
      float reverseDifference = initialTiltX - XTiltValue;
      float reverseTotal = initialTiltX + reverseDifference;
      XTiltValue = round(reverseTotal/1)*1;
      XTiltValue += 20;
      reversed = true;
    }

    if (XTiltValue > oldXTiltValue) { //stops water level value from increasing when a refill has not happened
      XTiltValue = oldXTiltValue;
    }

    //if the cup is empty
    if (XTiltValue == 0) {
      percentFull = XTiltValue; //XTiltValue == 0 here
      Particle.publish("Empty", percentFull, PRIVATE);
      sendServer(percentFull);

      //if the cup is empty, stay empty until refilled (it refills when a loud sound is made)
      while (XTiltValue == 0) {
        soundValue = measure(); //measure sound, check if its more than ambient sound level (within threshold)
        readMPU9150();          //// reads compass, accelerometer and gyroscope data

        float emptynewTiltX = getXTilt(ax, az);
        float emptyXTiltFraction = (emptynewTiltX / initialTiltX) * 100; //turn 360 degrees into percentage (90 degrees (vertically upright) = 100%)
        int emptyXTiltValue = round(emptyXTiltFraction/1)*1;  //rounds the float to an int to remove the decimal places

        //if the sound levels have increased by a set threshold, and the cup is tilted upright
        if(soundValue > soundState + 250 && emptyXTiltValue >= 95 && emptyXTiltValue <= 105)
        {
          Serial.println("SOUND DETECTED!");
          calibration = true;
          delay(1000); //delay 2 seconds before next calibration, to make sure we're back to ambient sound levels

          //sendServer("100");
          XTiltValue = 100; //breaks out of the while loop
          Particle.publish("Refilled!", percentFull, PRIVATE);
        }
        delay(100);
      }
    }

    percentFull = XTiltValue;

    if (firstLoop) {
      Particle.publish("Percent Full", percentFull, PRIVATE);
      firstLoop = false;
    }
    if (XTiltValue != oldXTiltValue && firstLoop == false) {
      Particle.publish("Percent Full", percentFull, PRIVATE);
      sendServer(percentFull);
    }

    oldXTiltValue = XTiltValue; //the next loop uses this loop's XTilt value in comparisons

    delay(500);

    // Power Down Sensors
    //digitalWrite(I2CEN, LOW);
    //digitalWrite(ALGEN, LOW);
}


//Tell the server when and where motion/sound was detected
void sendServer(String str)
{
  String send = str + String('%');

  //post string data into the server directly

  Serial.println("about to send cup fill percent " +send); //debug

  client.println("POST /webapp/sendper HTTP/1.1");
  client.println("HOST: sccug-330-03.lancs.ac.uk");
  client.print("Content-Length: ");
  client.println(send.length());
  client.println("Content-Type: text/plain");
  client.println();
  client.println(send);
}

/*read sound, return max-min
*/
int measure()
{
  unsigned int sampleWindow = 50; // Sample window width in milliseconds (50 milliseconds = 20Hz)
  unsigned long endWindow = millis() + sampleWindow;  // End of sample window

  unsigned int signalSample = 0;
  unsigned int signalMin = 4095; // Minimum is the lowest signal below which we assume silence
  unsigned int signalMax = 0; // Maximum signal starts out the same as the Minimum signal

  // collect data for milliseconds equal to sampleWindow
  while (millis() < endWindow) {
      signalSample = analogRead(SOUND);
      if (signalSample > signalMax) {
          signalMax = signalSample;  // save just the max levels
      }
      else if (signalSample < signalMin) {
          signalMin = signalSample;  // save just the min levels
      }
  }
  return signalMax - signalMin;
}

void readMPU9150()
{
    //// reads the MPU9150 sensor values. Values are read in order of temperature,
    //// compass, Gyro, Accelerometer

    tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;
    cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);  //Compass_X
    cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);  //Compass_Y
    cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);  //Compass_Z
    ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
    ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
    az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
    gx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
    gy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
    gz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
}

//// returns accelaration along x-axis, should be 0-1g
float getAccelX(float x)
{
  return x/pow(2,15)*ACCEL_SCALE;
}

//// returns accelaration along z-axis, should be 0-1g
float getAccelY(float y)
{
  return y/pow(2,15)*ACCEL_SCALE;
}

//// returns accelaration along z-axis should be 0-1g
float getAccelZ(float z)
{
  return z/pow(2,15)*ACCEL_SCALE;
}

//// returns the vector sum of the acceleration along x, y and z axes
//// in g units
float getAccelXYZ(float x, float y, float z)
{
  x = getAccelX(x);
  y = getAccelY(y);
  z = getAccelZ(z);

  return sqrt(x*x+y*y+z*z);
}

//// returns tilt along x axis in radians - uses accelerometer
float getXTilt(float accelX, float accelZ)
{
   float tilt = atan2(accelX,accelZ)*RAD_TO_DEGREES; //*RAD_TO_DEGREES;
   if(tilt < 0)
   {
      tilt = tilt+360.0;
   }

   return tilt;
}

//// returns tilt along y-axis in radians
float getYTilt(float accelY, float accelZ)
{
   float tilt = atan2(accelY,accelZ)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }

   return tilt;
}
