#include <SparkFunMPU9250-DMP.h>
#include "config.h"
#include "defines.h"

//IMU Variables
int acc_x = 0;
int acc_y = 0;
int acc_z = 0;
int gyro_x = 0;
int gyro_y = 0;
int gyro_z = 0;
int mag_x = 0;
int mag_y = 0;
int mag_z = 0;
MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class
long long imu_update_count = 0;
int sequence_number = 0;

//Timing Variables
long prev_time = 0;
long veryveryslowloop_timer = 0;
long veryslowloop_timer = 0;
long slowloop_timer = 0;
long mediumloop_timer = 0;
long fastloop_timer = 0;
long idle_counter = 0;
long loop_counter = 0;
double current_time = 0.0;
double recv_msg_timeout = 0.0;
bool time_sync_active = false;

//Uart Variables
char inData[32];
int message_index = 0;
bool message_ended = false;

void setup()
{
  initHardware(); 

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() ) 
  {
    SerialUSB.println("[IMU]: Failed to Initialize.");
    while (1) ;
  }
}

void loop()
{
  loop_counter++;
  
  long now = millis();
  long dt = now - prev_time;
  current_time = current_time += (double)(dt)/1000.0;
  recv_msg_timeout+=(double)(dt)/1000.0;
  prev_time = now;
  if((Serial1.available() > 0 ) and (message_ended == false))
  {
    inData[message_index] = Serial1.read();
    if(inData[message_index] == '*')
    {
      message_ended = true;
    }
    message_index++;
    
  }
  if((inData[0] == '$') and (message_ended == true)) //Message is ready to be processed
  {
    inData[message_index] = '\0';
    String str(inData);
    String timestr = str.substring(2,str.length()-1);
    time_sync_active = true;
    current_time = timestr.toDouble();
    recv_msg_timeout = 0.0;
    message_ended = false;
    message_index = 0;
    memset(inData,0,sizeof(inData));
  }
  else if(message_ended == true)
  {
    inData[message_index] = '\0';
    message_ended = false;
    message_index = 0;
    memset(inData,0,sizeof(inData));
  }
  if(true == true)
  {
    bool nothing_ran = true;
    fastloop_timer += dt;
    mediumloop_timer += dt;
    slowloop_timer += dt;
    veryslowloop_timer += dt;
    veryveryslowloop_timer += dt;
    if(fastloop_timer > FASTLOOP_RATE)
    {
      fastloop_timer = 0;
      run_fastloop(FASTLOOP_RATE);
      nothing_ran = false;
    }
    if(mediumloop_timer > MEDIUMLOOP_RATE)
    {
      mediumloop_timer = 0;
      run_mediumloop(MEDIUMLOOP_RATE);
      nothing_ran = false;
    }
    
    if(slowloop_timer > SLOWLOOP_RATE)
    {
      slowloop_timer = 0;
      run_slowloop(SLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(veryslowloop_timer > VERYSLOWLOOP_RATE)
    {
      veryslowloop_timer = 0;
      run_veryslowloop(VERYSLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(veryveryslowloop_timer > VERYVERYSLOWLOOP_RATE)
    {
      veryveryslowloop_timer = 0;
      run_veryveryslowloop(VERYVERYSLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(nothing_ran == true)
    {
      idle_counter++;
    }
  //delay(1);
  }
  if(loop_counter > 1000000)
  {
    loop_counter = idle_counter = 0;
  }
}
bool run_veryveryslowloop(long dt)
{

}
bool run_veryslowloop(long dt)
{
  if(DEBUG_PRINT >= 1)
  {
    SerialUSB.print("T:");
    SerialUSB.println(current_time);
    SerialUSB.print("[Status]: Idle Time: ");
    double idle_perc = 100.0*(double)idle_counter/(double)loop_counter;
    SerialUSB.print(idle_perc);
    SerialUSB.println(" %");
    SerialUSB.print("Update Count: ");
    SerialUSB.print((long)imu_update_count);
    SerialUSB.print(" Rate: ");
    SerialUSB.print(1000.0*(double)(imu_update_count)/((double)(millis())));
    SerialUSB.println(" Hz");
    SerialUSB.print("Time Sync: ");
    SerialUSB.println(time_sync_active);
  }
}

bool run_slowloop(long dt)
{
}
bool run_mediumloop(long dt)
{
  if(recv_msg_timeout > 2.0)
  {
    digitalWrite(HW_LED_PIN,LOW);
  }
  else
  {
    digitalWrite(HW_LED_PIN,!digitalRead(HW_LED_PIN));
  }
}
bool run_fastloop(long dt)
{
  if ( !imu.fifoAvailable() ) // If no new data is available
    return true;                   // return to the top of the loop

  // Read from the digital motion processor's FIFO
  if ( imu.dmpUpdateFifo() != INV_SUCCESS )
    return false; // If that fails (uh, oh), return to top

  // If enabled, read from the compass.
  if ( (imu.updateCompass() != INV_SUCCESS) )
    return false; // If compass read fails (uh, oh) return to top
  //Change orientation based on https://github.com/dgitz/icarus_rover_v2/wiki/DESIGN_REFERENCE 
  acc_x = -imu.ay;
  acc_y = imu.ax;
  acc_z = imu.az;
  gyro_x = imu.gy;
  gyro_y = imu.gx;
  gyro_z = imu.gz;
  mag_x = imu.mx;
  mag_y = imu.my;
  mag_z = imu.mz;
  sendIMUData();
  imu_update_count++;
  sequence_number++;
  if(sequence_number > 255)
  {
    sequence_number = 0;
  }
}
void sendIMUData(void)
{
  String imuLog = "$"; // Create a fresh line to log
  imuLog += String(current_time) + ","; // Add time to log string
  imuLog += String(sequence_number) + ",";
  imuLog += String(acc_x) + ",";
  imuLog += String(acc_y) + ",";
  imuLog += String(acc_z) + ",";
  imuLog += String(gyro_x) + ",";
  imuLog += String(gyro_y) + ",";
  imuLog += String(gyro_z) + ",";
  imuLog += String(mag_x) + ",";
  imuLog += String(mag_y) + ",";
  imuLog += String(mag_z) + "*";
  if(DEBUG_PRINT == 2)
  {
    SerialUSB.println(imuLog);
  }
  Serial1.println(imuLog);
 
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up DEBUG port
  SerialUSB.begin(115200);
  
  // Set up serial connection port
  Serial1.begin(115200);
  Serial1.flush();
   if(DEBUG_PRINT)
  {
    SerialUSB.println("Booting...");
  }
}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(IMU_GYRO_FSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
  imu.setAccelFSR(IMU_ACCEL_FSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF); 
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); 
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); 

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, DMP_SAMPLE_RATE);

  return true; // Return success
}
