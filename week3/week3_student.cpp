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

#define ROLL_MAX 45
#define ROLL_MIN -45
#define PITCH_MAX 45
#define PITCH_MIN -45
#define GYRO_RATE_MAX 300

FILE *f;

//add global variable
int pwm;

//add constants
#define PWM_MAX 1300
#define neutral_power 1100
#define frequency 25000000.0
#define LED0 0x6      
#define LED0_ON_L 0x6   
#define LED0_ON_H 0x7   
#define LED0_OFF_L 0x8    
#define LED0_OFF_H 0x9    
#define LED_MULTIPLYER 4  

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

float pitch_smooth = 0;
float roll_smooth = 0;

//associated globals
char command = 0;
char prev_command;
int prev_version;
int ver;
int heartbeat = 0;
int prev_heartbeat = 0;

struct timeval te_val;
long curr_time;
long last_heartbeat_time;



void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep = settings | 0x10;
      int wake  = settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
  int on_value=0;

  int time_on_us=900;
  uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

   time_on_us=1200;
   off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

   time_on_us=1000;
   off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

  wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
  wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
  delay(100);

}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}



//when cntrl+c pressed, kill motors

void trap(int signal)

{

  set_PWM(0,1000);
  delay(100);
  set_PWM(1,1000);
  delay(100);  
  set_PWM(2,1000);
  delay(100);  
  set_PWM(3,1000);  
  delay(100);

  fclose(f);
  printf("ending program\n\r");

   run_program=0;
}

void safety_check(){
  if (roll_smooth > ROLL_MAX || roll_smooth < ROLL_MIN){
    printf("too much roll!\n");
    trap(1);
  } else if (pitch_smooth > PITCH_MAX || pitch_smooth < PITCH_MIN){
    printf("too much pitch!\n");
    trap(1);
  } else if (abs(imu_data[0]) > GYRO_RATE_MAX || abs(imu_data[1]) > GYRO_RATE_MAX || abs(imu_data[2]) > GYRO_RATE_MAX){
    printf("to much gyro!!!\n");
    trap(1);
  }
}

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

void keyboard_check(){
      //to refresh values from shared memory first 
      Keyboard keyboard=*shared_memory;
      command = keyboard.key_press;
      ver = keyboard.version;
      heartbeat = keyboard.heartbeat;
      
      //check heartbeat 
      //get current time & set heartbeat timer to that
      gettimeofday(&te_val,NULL);
      curr_time=te_val.tv_sec*1000LL+te_val.tv_usec/1000;

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
}

void pid_update(){
  double pitch_error;
  double P = 2;
  double effort;
  int u;
  pitch_error = pitch_smooth - 0;
  effort = P * pitch_error;
  u = (int) effort;
  // groups: 0 & 1, 2 & 3
  printf("effort: %d\tangle: %f\n", u, pitch_smooth);
  set_PWM(3, neutral_power - u);
  set_PWM(2, neutral_power - u);
  set_PWM(1, neutral_power + u);
  set_PWM(0, neutral_power + u);

  fprintf(f, "%d,%d,%f\n", (neutral_power + u),(neutral_power - u),pitch_smooth);
}

int main (int argc, char *argv[])
{    
    //initialize keyboard
    setup_keyboard();
    signal(SIGINT, &trap); //when sign-int ctrl-c directs to this callback
    //setup and callibrate imu
    setup_imu();
    init_pwm();
    calibrate_imu();
 
    // command = keyboard.key_press;
    // prev_version = keyboard.version;
    // prev_heartbeat = keyboard.heartbeat;

    gettimeofday(&te_val,NULL);
    curr_time=te_val.tv_sec*1000LL+te_val.tv_usec/1000;
    last_heartbeat_time=te_val.tv_sec*1000LL+te_val.tv_usec/1000;

  
    init_motor(0);
    delay(100);
    
    init_motor(1);
    delay(100);
    
    init_motor(2);
    delay(100);

    init_motor(3);
    delay(100);
    printf("motor setup complete\n");
    f = fopen("P_PWM.csv", "w+");
    while(run_program == 1)
    {
      // keyboard_check();
      read_imu();      
      update_filter();   
      safety_check();
      pid_update();
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
    // printf("%10.5f \t %10.5f \t %10.5f \t %10.5f \t %10.5f\n", imu_data[0], imu_data[1], imu_data[2], roll_smooth, pitch_smooth);

  }
}

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

  pitch_smooth = pitch_curr;
  roll_smooth = roll_curr;

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
