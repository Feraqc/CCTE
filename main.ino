#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BMP280_DEV.h>      

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;
BMP280_DEV bmp280;

unsigned long time;
bool ign_flag = false;
bool gyro_cal = false;
//bool blinkState = false;
int led_state;
int led_ind[] = {6,7,8};
int ign = 3;
float temperature, pressure, altitude, reff;
bool bmp_reff;

//----------------------------------------------------------------------------------------------------//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//------------------------------------------------------------------------------------------------------//

void setup() {

   pinMode(led_ind[0],OUTPUT);
   pinMode(led_ind[1],OUTPUT);
   pinMode(led_ind[2],OUTPUT);
   pinMode(ign,OUTPUT);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(115200);
    while (!Serial); 
    sensors_innit();                     // funcion que inicializa los sensores 


}

void loop() {

    time = millis();
    check_sensors();

    get_sensors_data();

    (wait_gyro_calibration())?set_state(2):set_state(1);

    if(!ign_flag){
      parachute_deployment();
    }

}

void sensors_innit(){
    set_state(0);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);


        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    bmp280.begin(BMP280_I2C_ALT_ADDR);              // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE 
    bmp280.setPresOversampling(OVERSAMPLING_X16);    // Set the pressure oversampling to X4
    bmp280.setTempOversampling(OVERSAMPLING_X16);    // Set the temperature oversampling to X1
    bmp280.setIIRFilter(IIR_FILTER_16);              // Set the IIR filter to setting 4
    bmp280.setTimeStandby(TIME_STANDBY_05MS);     // Set the standby time to 2 seconds
    bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

    set_state(1);
}

void set_state(int led_state){

  switch (led_state) {
    
    case 0: //sensores no inicializados
      digitalWrite(led_ind[0],1);
      digitalWrite(led_ind[1],0);
      digitalWrite(led_ind[2],0);
      break;

    case 1: // calibracion de sensores
      digitalWrite(led_ind[0],0);    
      digitalWrite(led_ind[1],1);
      digitalWrite(led_ind[2],0);

      break;

    case 2: // comienzo de adquisicion de datos
      digitalWrite(led_ind[0],0);
      digitalWrite(led_ind[1],0);
      digitalWrite(led_ind[2],1);
      break;
    
    case 3: // adquisicion de datas exitosa 
      break;

  }
}

void parachute_deployment(){
  
  if(abs(ypr[1] * 180/M_PI) >= 80.0||abs(ypr[2] * 180/M_PI) >= 80.0){
    ign_flag = true;

    digitalWrite(ign,1);
    delay(2500);
    digitalWrite(ign,0);
  
  }
}

bool wait_gyro_calibration(){
  if(time > 20000){
      if(abs(ypr[1] * 180/M_PI) < 1.0 && abs(ypr[2] * 180/M_PI) < 1.0){
        gyro_cal = true;
      }
  }
  return gyro_cal;
}

void check_sensors(){
  
  if(!mpu.testConnection()){                            
    Serial.println("Connection failed");
    set_state(0); 
  }

}

void get_sensors_data(){

      if(!mpu.testConnection()){                            
        Serial.println("Connection failed");
        set_state(0);
      }

      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }

      if (bmp280.getMeasurements(temperature, pressure, altitude)){ 
        
        if(!bmp_reff){
          reff = altitude;
          bmp_reff = true;
        }
      }

          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.print(aaReal.z);

          Serial.print("\typr\t");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
          Serial.print(ypr[2] * 180/M_PI);

          Serial.print("\t");
          Serial.print(ign_flag);
          Serial.print("\t");
          Serial.print(gyro_cal);
          Serial.print("\t");

        Serial.print(temperature);
        Serial.print("\t");  
        Serial.print(F("*C   "));
        Serial.print("\t");
        Serial.print(pressure);    
        Serial.print(F("hPa   "));
        Serial.print("\t");
        Serial.print(altitude - reff);
        Serial.print(F("m"));  
        Serial.print("\n");
    
}

void write_data(){
  ;

  
}
