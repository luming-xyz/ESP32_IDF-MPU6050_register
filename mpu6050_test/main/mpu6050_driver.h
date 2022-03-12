#include "esp_err.h"  
#include "driver/i2c.h" 

#define MPU6050_I2C_SDA 19
#define MPU6050_I2C_SCL 18
#define MPU6050_I2C_PORT_NUM 0
#define MPU6050_I2C_FREQ 400000
#define MPU6050_ADDR 0x68  //器件地址： b110100(AD0) 

#define WRITE_BIT I2C_MASTER_WRITE  //I2C master write 
#define READ_BIT I2C_MASTER_READ    //I2C master read 
#define ACK_CHECK_EN 0x1            //I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           //I2C master will not check ack from slave 
#define ACK_VAL 0x0                 //I2C ack value 
#define NACK_VAL 0x1                //I2C nack value    

typedef struct accel_out_tag
{
    int16_t accel_xout;
    int16_t accel_yout; 
    int16_t accel_zout; 
}accel_out_t;

typedef struct temp_out_tag
{
    int16_t temp_xout; 
}temp_out_t;

typedef struct gyro_out_tag
{
    int16_t gyro_xout; 
    int16_t gyro_yout; 
    int16_t gyro_zout; 
}gyro_out_t;

typedef struct measurement_out_tag
{
    accel_out_t accel_out;
    temp_out_t temp_out;
    gyro_out_t gyro_out;
}measurement_out_t;


/**
* @brief 初始化 mpu6050
*/
esp_err_t mpu6050_init();

/**
* @brief 读取加速度计、温度和陀螺仪数据
*/
measurement_out_t mpu6050_get_value();