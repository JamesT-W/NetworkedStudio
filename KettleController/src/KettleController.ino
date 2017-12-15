#include "MPU9150.h"
#include "Si1132.h"
#include "Si70xx.h"
#include "math.h"


//// ***************************************************************************
//// ***************************************************************************

//// Initialize application variables
#define RAD_TO_DEGREES 57.2957795131
#define DEG_TO_RADIANS 0.0174533
#define PI 3.1415926535
#define ACCEL_SCALE 2 // +/- 2g
#define COMP_I2C_ADDRESS  0x0C;      //change Adress to Compass  // R/W

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R

int SENSORDELAY = 500;  //// 500; //3000; // milliseconds (runs x1)
int EVENTSDELAY = 1000; //// milliseconds (runs x10)
int OTAUPDDELAY = 7000; //// milliseconds (runs x1)
int SLEEP_DELAY = 0;    //// 40 seconds (runs x1) - should get about 24 hours on 2100mAH, 0 to disable and use RELAX_DELAY instead
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

float pitch = 0;
float roll = 0;
float yaw = 0;

float compassX = 0;
float compassY = 0;
float compassZ = 0;

float originalX = 0;
float originalY = 0;
float originalZ = 0;

int mag[3]; //*******

Si1132 si1132 = Si1132();

// I2C address 0x69 could be 0x68 depends on your wiring.
int MPU9150_I2C_ADDRESS = 0x68;

TCPClient client;  //used to connect to the server
const char serverURL[] = "sccug-330-03.lancs.ac.uk"; //ip address
const int serverPort = 80;  //port

TCPClient kettleTCP;  //used to connect to the server
const char kettleIP[] = "192.168.0.103"; //ip address
const int kettlePort = 2000;  //port

LEDStatus blinkYellow(RGB_COLOR_YELLOW, LED_PATTERN_SOLID, LED_SPEED_SLOW);

int previousTemp = 0;

//// ***************************************************************************
//// ***************************************************************************


///SYSTEM_MODE(SEMI_AUTOMATIC);

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
    MPU9150_I2C_ADDRESS = 0x68;
    MPU9150_writeSensor(0x6B,0x01);            // PLL with X axis gyroscope reference and disable sleep mode
    MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0); // Clear the 'sleep' bit to start the sensor.

    initialiseMPU9150();
    readMPU9150();          // reads compass, accelerometer and gyroscope

    originalX = getCompassX(cx);
    originalY = getCompassY(cy);
    originalZ = getCompassZ(cz);

    //blinkYellow.setActive(true);

    //Particle.subscribe("Entering", sendKettleOnDoor);
	  //Particle.subscribe("Empty", sendKettleOnCup);

    //connectVM();
}

//attempt to connect to VMserver, blink red if unable to
/*void connectVM(){
  interrupts();
  if(client.connect(serverURL, serverPort))
  {
    blinkYellow.setActive(false);
  }
  noInterrupts();
}*/

void initialiseMPU9150()
{
  ACCELOK = mpu9150.begin(mpu9150._addr_motion); // Initialize MPU9150
  // Clear the 'sleep' bit to start the sensor.
  mpu9150.writeSensor(mpu9150._addr_motion, 0x6B, 0x01);
  mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);


  if (ACCELOK)
  {
      //mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);

      //set compass
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //Powerdown mode
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x0F); //Self test
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //Powerdown mode

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

      delay(100);
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

    //// allows sensors time to warm up
    delay(SENSORDELAY);

    /*if(client.connected() != true)
    {
      blinkYellow.setActive(true);
      Serial.println("Connection to server lost");
      connectVM();
    }*/

    readMPU9150();          // reads compass, accelerometer and gyroscope

    compassX = getCompassX(cx);
    compassY = getCompassY(cy);
    compassZ = getCompassZ(cz);

    String compassString = "";
    compassString = compassString+"X: "+compassX+" Y: "+compassY+" Z: "+compassZ;

    String originalCompassString = "";
    originalCompassString = originalCompassString+"X: "+originalX+" Y: "+originalY+" Z: "+originalZ;

    //determines which temp value it should send depending on how much the photon has been tilted and in which direction
    if(compassX > originalX + 8.0) {
      if (previousTemp != 65) {
        sendKettleTemp(65);
        sendServer(65);
        previousTemp = 65;
      }
    }
    else if(compassX > originalX + 4.0) {
      if (previousTemp != 80) {
        sendKettleTemp(80);
        sendServer(80);
        previousTemp = 80;
      }
    }
    else if(compassX < originalX - 8.0) {
      if (previousTemp != 100) {
        sendKettleTemp(100);
        sendServer(100);
        previousTemp = 100;
      }
    }
    else if(compassX < originalX - 4.0) {
      if (previousTemp != 95) {
        sendKettleTemp(95);
        sendServer(95);
        previousTemp = 95;
      }
    }

    Particle.publish("new", compassString, PRIVATE);
    Particle.publish("original", originalCompassString, PRIVATE);
    delay(1000);
}

//Tell the server the temperature value the kettle is set at
void sendServer(int temp)
{
  String send = "" + String(temp);
  //post string data into the server directly
  Serial.println("about to send post request"); //debug
  Serial.println(send);

  client.println("POST /webapp/sendkettle HTTP/1.1");
  client.println("HOST: sccug-330-03.lancs.ac.uk");
  client.print("Content-Length: ");
  client.println(send.length());
  client.println("Content-Type: text/plain");
  client.println();
  client.println(send);
}

//sends the string to the kettle to change the temperature to the correct value
void sendKettleTemp(int temp)
{
  String kettleTemps[4];
  kettleTemps[0] = "set sys output 0x200";  //65 degrees
  kettleTemps[1] = "set sys output 0x4000"; //80 degrees
  kettleTemps[2] = "set sys output 0x2";    //95 degrees
  kettleTemps[3] = "set sys output 0x80";   //100 degrees

  if (kettleTCP.connect(kettleIP, kettlePort)) {
    if (temp == 65) {
      Particle.publish("65 Degrees", kettleTemps[0], PUBLIC);
      kettleTCP.println(kettleTemps[0]);
    }
    else if (temp == 80) {
      Particle.publish("80 Degrees", kettleTemps[1], PUBLIC);
      kettleTCP.println(kettleTemps[1]);
    }
    else if (temp == 95) {
      Particle.publish("95 Degrees", kettleTemps[2], PUBLIC);
      kettleTCP.println(kettleTemps[2]);
    }
    else if (temp == 100) {
      Particle.publish("100 Degrees", kettleTemps[3], PUBLIC);
      kettleTCP.println(kettleTemps[3]);
    }
  }
  else {
    Particle.publish("connection failed", "connectionFailed", PUBLIC);
  }

  delay(500);
}

//tells myHandler that the Door is turning the Kettle on
void sendKettleOnDoor(const char *event, const char *data)
{
  Particle.publish("FIRST LOOP", data, PUBLIC);
  myHandler("Door");
}

//tells myHandler that the Cup is turning the Kettle on
void sendKettleOnCup(const char *event, const char *data)
{
  myHandler("Cup");
}

//Turns the Kettle on
void myHandler(String triggeredBy)
{
  /* Particle.subscribe handlers are void functions, which means they don't return anything.
  They take two variables-- the name of your event, and any data that goes along with your event.
  */
  Serial.println("in loop");

  String kettleOn = "set sys output 0x4";

  Particle.publish("in loop", kettleOn, PUBLIC);

  if (kettleTCP.connect(kettleIP, kettlePort)) {
    if (triggeredBy.equals("Door")) {
      Serial.println("Door triggered kettle on!");
      Particle.publish("Kettle on", kettleOn, PUBLIC);
      kettleTCP.println(kettleOn);
    }
    else if (triggeredBy.equals("Cup")) {
      Serial.println("Cup triggered kettle on!");
      Particle.publish("Kettle on", kettleOn, PUBLIC);
      kettleTCP.println(kettleOn);
    }
    else {
      Serial.print("Winner is not Door or Cup");
    }
  }
  else {
    Particle.publish("connection failed", "connectionFailed", PUBLIC);
  }
}

void readMPU9150()
{
    //// reads the MPU9150 sensor values. Values are read in order of temperature,
    //// compass, Gyro, Accelerometer

    tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;

    //cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
    //cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
    //cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);

    cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);
    cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);
    cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);

    ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
    ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
    az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);

    gx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
    gy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
    gz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
}

int readWeatherSi7020()
{
    Si70xx si7020;
    Si7020OK = si7020.begin(); //// initialises Si7020

    if (Si7020OK)
    {
        Si7020Temperature = si7020.readTemperature();
        Si7020Humidity = si7020.readHumidity();
    }

    return Si7020OK ? 2 : 0;
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

float getRoll(float accelX, float accelY, float accelZ)
{
  return atan2(accelY , sqrt(accelX*accelX+accelZ*accelZ))*RAD_TO_DEGREES;
}


float getPitch(float accelX, float accelY, float accelZ)
{
  return atan2(accelX , sqrt(accelY*accelY+accelZ*accelZ))*RAD_TO_DEGREES;
}


//// returns tilt along x axis in radians - uses accelerometer
float getXTilt(float accelX, float accelZ)
{


   float tilt = atan2(accelX,accelZ)*RAD_TO_DEGREES; //*RAD_TO_DEGREES;
   if(tilt < 0)
   {
      tilt = tilt+360.0;
   }
   return tilt*DEG_TO_RADIANS;
  //return tilt;
}

//// returns tilt along y-axis in radians
float getYTilt(float accelY, float accelZ)
{
   float tilt = atan2(accelY,accelZ)*RAD_TO_DEGREES;
   if(tilt < 0)
   {
     tilt = tilt+360.0;
   }
   return tilt*DEG_TO_RADIANS;
   //return tilt;
}

float getCompassX(float x)
{
  return x *0.3;
}

float getCompassY(float y)
{
  return y*0.3;
}

float getCompassZ(float z)
{
  return z*0.4;
}

float readSoundLevel()
{
    unsigned int sampleWindow = 50; // Sample window width in milliseconds (50 milliseconds = 20Hz)
    unsigned long endWindow = millis() + sampleWindow;  // End of sample window

    unsigned int signalSample = 0;
    unsigned int signalMin = 4095; // Minimum is the lowest signal below which we assume silence
    unsigned int signalMax = 0; // Maximum signal starts out the same as the Minimum signal

    // collect data for milliseconds equal to sampleWindow
    while (millis() < endWindow)
    {
        signalSample = analogRead(SOUND);
        if (signalSample > signalMax)
        {
            signalMax = signalSample;  // save just the max levels
        }
        else if (signalSample < signalMin)
        {
            signalMin = signalSample;  // save just the min levels
        }
    }

    //SOUNDV = signalMax - signalMin;  // max - min = peak-peak amplitude
    SOUNDV = mapFloat((signalMax - signalMin), 0, 4095, 0, 3.3);



    //return 1;
    return SOUNDV;
}

void readWeatherSi1132()
{

    si1132.begin(); //// initialises Si1132
    Si1132UVIndex = si1132.readUV() *0.01;
    Si1132Visible = si1132.readVisible();
    Si1132InfraRd = si1132.readIR();
}


int MPU9150_readMagSensor(int addrL, int addrH)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);//1
  Wire.endTransmission(false);//0
  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);//1
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();
  return (int16_t)((H<<8)+L);
}


int MPU9150_readSensor(int addr)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  return Wire.endTransmission(true);
}

int MPU9150_writeSensor(int addr,uint8_t* data,uint8_t length, bool sendStop)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data,length);
  return Wire.endTransmission(sendStop);// Returns 0 on success
}
