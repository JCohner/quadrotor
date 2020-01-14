#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

FILE *f;

//Keyboard interfacing globals
struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
Keyboard* shared_memory; 
int run_program=1;

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float PI = 3.14159;
int callib_flag = 0; 

//function to add
void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

//when cntrl+c pressed, kill motors

void trap(int signal)

{

   
 
   printf("ending program\n\r");

   run_program=0;
}


int main (int argc, char *argv[])
{
    //initialize keyboard
    setup_keyboard();
    signal(SIGINT, &trap); //when sign-int ctrl-c directs to this callback
    //setup and callibrate imu
    setup_imu();
    calibrate_imu();
    // char val = argv[1][0];
    // if (val =='r'){
    //   f = fopen("roll.csv", "w+");
    // }  else if (val == 'p'){
    //   f = fopen("pitch.csv", "w+");
    // } else{
    //   printf("ya goofed\n");
    // }
 
    char command = 0;
    char prev_command;
    int prev_version;
    int ver;
    int heartbeat = 0;
    int prev_heartbeat = 0;

    Keyboard keyboard=*shared_memory;
    command = keyboard.key_press;
    prev_version = keyboard.version;
    prev_heartbeat = keyboard.heartbeat;

    struct timeval te;
    gettimeofday(&te,NULL);
    long curr_time=te.tv_sec*1000LL+te.tv_usec/1000;
    long last_heartbeat_time=te.tv_sec*1000LL+te.tv_usec/1000;


    while(run_program == 1)
    {
      //to refresh values from shared memory first 
      Keyboard keyboard=*shared_memory;
      command = keyboard.key_press;
      ver = keyboard.version;
      heartbeat = keyboard.heartbeat;
      
      //check heartbeat 
      //get current time & set heartbeat timer to that
      gettimeofday(&te,NULL);
      curr_time=te.tv_sec*1000LL+te.tv_usec/1000;

      // printf("heartbeat: %d\nprev_heartbeat: %d", heartbeat, prev_heartbeat);
      if (heartbeat != prev_heartbeat){
        last_heartbeat_time=curr_time;
        prev_heartbeat = heartbeat;
      }
      if ((curr_time - last_heartbeat_time) > 250){
        printf("timeout\n");
        trap(1);
      }

      if (command != 0 && (ver != prev_version)){
        printf("%c\n",keyboard.key_press);
        prev_version = ver;
        if (command == 32){
          trap(1);
        }
      } 
      read_imu();      
      update_filter();   
    }
    return 0;
}

void calibrate_imu()
{
  float callib_data[6] = {0,0,0,0,0,0};
  int flag = 0;
  for (int i = 0; i < 1000; i++){
    read_imu();      
    callib_data[0] += imu_data[0]/1000.0;
    callib_data[1] += imu_data[1]/1000.0;
    callib_data[2] += imu_data[2]/1000.0; 
    callib_data[3] += atan2(imu_data[3], imu_data[5])/1000.0;
    callib_data[4] += atan2(imu_data[4], imu_data[5])/1000.0;
    callib_data[5] += imu_data[5]/1000.0; 
  }

  x_gyro_calibration=callib_data[0];
  y_gyro_calibration=callib_data[1];
  z_gyro_calibration=callib_data[2];
  
  roll_calibration= callib_data[3];
  pitch_calibration= callib_data[4];
  /*
  accel_z_calibration=??
  */
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
callib_flag = 1;

}

void read_imu()
{
  int address= 0x3B;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;   

  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  ax = (vw / 16384.0);
  // printf("%d\n",vw);
  // printf("%f\n",gx);          
  imu_data[3]=ax;//  todo: convert vw from raw values to "g's"
  
  
  address=0x3D;//todo: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  ay = (vw / 16384.0);          
  imu_data[4]=-ay;//Todo: convert vw from raw valeus to "g's"
  
  
  address=0x3F;//todo: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  az = (vw / 16384.0);          
  imu_data[5]=-az;//todo: convert vw from raw values to g's
  
  
  address=0x43;//todo: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  float gx = vw/32768.0 * 500;          
  // float gx_delta = dt * gx;
  imu_data[0]=-x_gyro_calibration+gx;////todo: convert vw from raw values to degrees/second
  
  address=0x45;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  float gy = vw/32768.0 * 500;         
 imu_data[1]= - y_gyro_calibration+gy;////todo: convert vw from raw values to degrees/second
  
  address=0x47;////todo: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  float gz = vw/32768.0 * 500;           
  imu_data[2]=-z_gyro_calibration+gz;////todo: convert vw from raw values to degrees/second
  
  roll_angle = (atan2(imu_data[3],imu_data[5]) - roll_calibration) / PI * 180;
  pitch_angle = (atan2(imu_data[4],imu_data[5]) - pitch_calibration) / PI * 180;

  if (callib_flag){
    // printf("%10.5f \t %10.5f \t %10.5f \t %10.5f \t %10.5f\n", imu_data[0], imu_data[1], imu_data[2], roll_angle, pitch_angle);
  }
}

float roll_accel = roll_angle;
float roll_next;
float A = 0.02;
float roll_curr = 0;
float roll_gyro_delta;
float roll_sum = 0;

float pitch_accel = pitch_angle;
float pitch_next;
float B = 0.02;
float pitch_curr = 0;
float pitch_gyro_delta;
float pitch_sum = 0;

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  //comp. filter for roll, pitch here:
  /*Roll*/  
  roll_accel = roll_angle;
  roll_gyro_delta = imu_data[1] * imu_diff;
  roll_sum += roll_gyro_delta;
  // printf("accel: %f\n", roll_accel);
  roll_next = roll_accel * A + (1-A) * (roll_gyro_delta + roll_curr);
  roll_curr = roll_next;
  
  /*Pitch*/
  pitch_accel = pitch_angle;
  pitch_gyro_delta = imu_data[0] * imu_diff;
  pitch_sum += pitch_gyro_delta;
  // printf("accel: %f\n", pitch_accel);
  pitch_next = pitch_accel * B + (1-B) * (pitch_gyro_delta + pitch_curr);
  pitch_curr = pitch_next;

  // /*File IO*/
  // if (val == 'r'){
  //   fprintf(f, "%f, %f, %f, %ld\n", roll_accel, roll_sum, roll_curr, time_curr);
  //   printf("roll:\naccel: %f \tsum: %f\tfilt: %f\n", roll_accel, roll_sum, roll_curr);

  // } else if (val == 'p'){
  //   fprintf(f, "%f, %f, %f, %ld\n", pitch_accel, pitch_sum, pitch_curr, time_curr);
  //   printf("pitch:\naccel: %f \tsum: %f\tfilt: %f\n", pitch_accel, pitch_sum, pitch_curr);
  // }

}


int setup_imu()
{
  wiringPiSetup ();
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}
