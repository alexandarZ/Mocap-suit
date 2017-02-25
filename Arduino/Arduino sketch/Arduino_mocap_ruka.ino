#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define BLENDER_3D

MPU6050 MPU_A(0x69);
MPU6050 MPU_B(0x68);

/* DMP ready variables */
bool mpuA_dmp_ready = false;
bool mpuB_dmp_ready = false;

/* Sensor connection status */
uint8_t mpuA_connected;
uint8_t mpuB_connected;

/* Sensor interrupt status */
uint8_t  mpuA_int_status;
uint8_t  mpuB_int_status;

/* Sensor DMP status */
uint8_t  mpuA_dma_status;
uint8_t  mpuB_dma_status;

/* Sensor packet size */
uint16_t mpuA_packet_size;
uint16_t mpuB_packet_size;

/* Sensor packet size */
uint16_t mpuA_fifo_count;
uint16_t mpuB_fifo_count;

/* Sensor FIFO buffers */
uint8_t  mpuA_fifo_buffer[64];
uint8_t  mpuB_fifo_buffer[64];

/* Sensor interrupt variables */
volatile bool mpuA_interrupt = false;
volatile bool mpuB_interrupt = false;

/* Sensor read data */
volatile bool mpuA_read_data = false;
volatile bool mpuB_read_data = false;

/* Quaternions */
Quaternion q1, q2; // [w, x, y, z]

/*  Sensor packet
    0  - $    - Begining of packet
    1  - 0x01 - Sensor 1 data [info]
    2  - Sensor A Qw high part
    3  - Sensor A Qw low  part
    4  - Sensor A Qx high part
    5  - Sensor A Qx low  part
    6  - Sensor A Qy high part
    7  - Sensor A Qy low  part
    8  - Sensor A Qz high part
    9  - Sensor A Qz low  part
    10 - 0x02  - Sensor 2 data [info]
    11 - Sensor B Qw high part
    12 - Sensor B Qw low  part
    13 - Sensor B Qx high part
    14 - Sensor B Qx low  part
    15 - Sensor B Qy high part
    16 - Sensor B Qy low  part
    17 - Sensor B Qz high part
    18 - Sensor B Qz low  part
    19 - 0x03  - Packet info data [info]
    20 - Packet counter
    21 - \r
    22 - \n
*/

uint8_t sensor_data_packet[23] = { '$', 0x01, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x02, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x03, 0x00, '\r', '\n'};

float mpu_b_X_accell_offset = -1839;
float mpu_b_Y_accell_offset = 3365;
float mpu_b_Z_accell_offset = 1463;
float mpu_b_X_gyro_offset   = 21;
float mpu_b_Y_gyro_offset   = 41;
float mpu_b_Z_gyro_offset   = -7;

float mpu_a_X_accell_offset = -2399;
float mpu_a_Y_accell_offset = 2042;
float mpu_a_Z_accell_offset = 1751;
float mpu_a_X_gyro_offset   = 70;
float mpu_a_Y_gyro_offset   = -43;
float mpu_a_Z_gyro_offset   = -37;

unsigned long reset_time;
unsigned long reset_prev_time = 0;
unsigned long reset_interval = 20000; //20s

void dmpDataReady()
{
  //    mpuA_int_status = MPU_A.getIntStatus();
  //    mpuB_int_status = MPU_B.getIntStatus();
  //
  //    if(mpuA_int_status & 0x02)
  //    {
  //      mpuA_interrupt = true;
  //    }
  //
  //    if(mpuB_int_status > 0)
  //    {
  //      mpuB_interrupt = true;
  //    }
}

void check_sensor_connections()
{
  /* Checking sensor connections */

  Serial.println("Testing device connections");

  if (MPU_A.testConnection())
  {
    mpuA_connected = 1;
    Serial.println("Sensor A connected - TRUE");
  }
  else
  {
    mpuA_connected = 0;
    Serial.println("Sensor A connected - FALSE");
  }

  if (MPU_B.testConnection())
  {
    mpuB_connected = 1;
    Serial.println("Sensor B connected - TRUE");
  }
  else
  {
    mpuB_connected = 0;
    Serial.println("Sensor B connected - FALSE");
  }
}

void initialize_sensor_dma(uint8_t device)
{
  /* Sensor A */

  if (device == 0)
  {
    Serial.println("Sensor A DMP initialization");

    mpuA_dma_status = MPU_A.dmpInitialize();

    /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_A.setZAccelOffset(mpu_a_Z_accell_offset);
    MPU_A.setXGyroOffset(mpu_a_X_gyro_offset);
    MPU_A.setYGyroOffset(mpu_a_Y_gyro_offset);
    MPU_A.setZGyroOffset(mpu_a_Z_gyro_offset);

    if (mpuA_dma_status == 0)
    {
      Serial.println("Sensor A DMP initialized");
      Serial.println("Enabling DMP");

      MPU_A.setDMPEnabled(true);

      Serial.println("Sensor A Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuA_int_status = MPU_A.getIntStatus();

      Serial.println("Sensor A DMP Ready, waiting for interrupt");
      mpuA_dmp_ready = true;

      mpuA_packet_size = MPU_A.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor A DMP initialization failed");
      if (mpuA_dma_status == 1)
      {
        Serial.println("Sensor A - Initial memory load failed!");
      }
      else if (mpuA_dma_status == 2)
      {
        Serial.println("Sensor A - DMP configuration updates failed!");
      }
    }
  }

  /* Sensor B */
  if (device == 1)
  {
    Serial.println("Sensor B DMP initialization");

    mpuB_dma_status = MPU_B.dmpInitialize();

    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_B.setZAccelOffset(mpu_b_Z_accell_offset);
    MPU_B.setXGyroOffset(mpu_b_X_gyro_offset);
    MPU_B.setYGyroOffset(mpu_b_Y_gyro_offset);
    MPU_B.setZGyroOffset(mpu_b_Z_gyro_offset);

    if (mpuB_dma_status == 0)
    {
      Serial.println("Sensor B DMP initialized");
      Serial.println("Enabling DMP");

      MPU_B.setDMPEnabled(true);

      Serial.println("Sensor B Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuB_int_status = MPU_B.getIntStatus();

      Serial.println("Sensor B DMP Ready, waiting for interrupt");
      mpuB_dmp_ready = true;

      mpuB_packet_size = MPU_B.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor B DMP initialization failed");
      if (mpuB_dma_status == 1)
      {
        Serial.println("Sensor B - Initial memory load failed!");
      }
      else if (mpuB_dma_status == 2)
      {
        Serial.println("Sensor B - DMP configuration updates failed!");
      }
    }
  }

}

void sensor_send_data_to_pc()
{
  if (mpuA_read_data && mpuB_read_data)
  {
    MPU_A.dmpGetQuaternion(&q1, mpuA_fifo_buffer);
    MPU_B.dmpGetQuaternion(&q2, mpuB_fifo_buffer);

#ifdef DEBUG
    Serial.print("Sensor A:\t");
    Serial.print(q1.w);
    Serial.print("\t");
    Serial.print(q1.x);
    Serial.print("\t");
    Serial.print(q1.y);
    Serial.print("\t");
    Serial.print(q1.z);
    Serial.print("\t");
    Serial.print("Sensor B:\t");
    Serial.print(q2.w);
    Serial.print("\t");
    Serial.print(q2.x);
    Serial.print("\t");
    Serial.print(q2.y);
    Serial.print("\t");
    Serial.println(q2.z);
#endif

#ifdef BLENDER_3D

    /* Sensor A data */
    sensor_data_packet[2] = mpuA_fifo_buffer[0];
    sensor_data_packet[3] = mpuA_fifo_buffer[1];
    sensor_data_packet[4] = mpuA_fifo_buffer[4];
    sensor_data_packet[5] = mpuA_fifo_buffer[5];
    sensor_data_packet[6] = mpuA_fifo_buffer[8];
    sensor_data_packet[7] = mpuA_fifo_buffer[9];
    sensor_data_packet[8] = mpuA_fifo_buffer[12];
    sensor_data_packet[9] = mpuA_fifo_buffer[13];

    /* Sensor B data */
    sensor_data_packet[11] = mpuB_fifo_buffer[0];
    sensor_data_packet[12] = mpuB_fifo_buffer[1];
    sensor_data_packet[13] = mpuB_fifo_buffer[4];
    sensor_data_packet[14] = mpuB_fifo_buffer[5];
    sensor_data_packet[15] = mpuB_fifo_buffer[8];
    sensor_data_packet[16] = mpuB_fifo_buffer[9];
    sensor_data_packet[17] = mpuB_fifo_buffer[12];
    sensor_data_packet[18] = mpuB_fifo_buffer[13];

    Serial.write(sensor_data_packet, 23);

    /* Increment packet counter */
    sensor_data_packet[20]++;
    
#endif

    mpuA_read_data = false;
    mpuB_read_data = false;
  }
}

void setup() {

  /* Initialize I2C */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  /* Setup UART */
  Serial.begin(115200);
  while (!Serial);

  /* Initialize sensors */
  Serial.println(F("Initializing I2C devices..."));
  MPU_A.initialize();
  MPU_B.initialize();

  /* Check connections */
  check_sensor_connections();

  /* Configure DMP */
  initialize_sensor_dma(0); //Sensor A
  initialize_sensor_dma(1); //Sensor B

  /* Sensors connection error handling */
  if (!mpuA_connected || !mpuB_connected)
  {
    Serial.println("ERROR: Both sensors must be connected!");

    while (1)
    {

    }
  }

  /* Sensors DMP error handling */
  if (!mpuA_dmp_ready || !mpuB_dmp_ready)
  {
    Serial.println("ERROR: Both sensors must have DMP initialized!");

    while (1)
    {

    }
  }
}

void loop() {

  /* Check interrupt status */
  mpuA_int_status = MPU_A.getIntStatus();
  mpuB_int_status = MPU_B.getIntStatus();

  /* Check FIFO count */
  mpuA_fifo_count = MPU_A.getFIFOCount();
  mpuB_fifo_count = MPU_B.getFIFOCount();

  /* MPU A read data */

  // FIFO overflow
  if ((mpuA_int_status & 0x10) || mpuA_fifo_count == 1024)
  {
    MPU_A.resetFIFO();
  }
  // New data
  else if (mpuA_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuA_fifo_count < mpuA_packet_size) {
      mpuA_fifo_count = MPU_A.getFIFOCount();
    }

    MPU_A.getFIFOBytes(mpuA_fifo_buffer, mpuA_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuA_fifo_count -= mpuA_packet_size;

    mpuA_read_data = true;
  }

  /* MPU B read data */

  // FIFO overflow
  if ((mpuB_int_status & 0x10) || mpuB_fifo_count == 1024)
  {
    MPU_B.resetFIFO();
  }
  // New data
  else if (mpuB_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuB_fifo_count < mpuB_packet_size) {
      mpuB_fifo_count = MPU_B.getFIFOCount();
    }

    MPU_B.getFIFOBytes(mpuB_fifo_buffer, mpuB_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuB_fifo_count -= mpuB_packet_size;

    mpuB_read_data = true;
  }

  /* Send data to PC */
  sensor_send_data_to_pc();

  /* Reset offsets if timeout */
  reset_time = millis();

  if(reset_time - reset_prev_time >= reset_interval)
  {
    reset_prev_time = reset_time;

    //MPU_A.resetFIFO();
    
  /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_A.setZAccelOffset(mpu_a_Z_accell_offset);
    MPU_A.setXGyroOffset(mpu_a_X_gyro_offset);
    MPU_A.setYGyroOffset(mpu_a_Y_gyro_offset);
    MPU_A.setZGyroOffset(mpu_a_Z_gyro_offset);

    //MPU_B.resetFIFO();
    
    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_B.setZAccelOffset(mpu_b_Z_accell_offset);
    MPU_B.setXGyroOffset(mpu_b_X_gyro_offset);
    MPU_B.setYGyroOffset(mpu_b_Y_gyro_offset);
    MPU_B.setZGyroOffset(mpu_b_Z_gyro_offset);
  }
}
