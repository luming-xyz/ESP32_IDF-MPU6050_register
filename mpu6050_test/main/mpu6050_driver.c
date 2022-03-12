#include "mpu6050_driver.h"  


static const uint8_t mpu6050_init_cmd[11][2] = {
    {0x6B, 0x80}, // PWR_MGMT_1, DEVICE_RESET  
    // need wait 
    {0x6B, 0x00}, // cleat SLEEP
    {0x1B, 0x18}, // Gyroscope Full Scale Range = ± 2000 °/s
    {0x1C, 0x00}, // Accelerometer Full Scale Range = ± 2g 
    {0x38, 0x00}, // Interrupt Enable.disenable 
    {0x6A, 0x00}, // User Control.auxiliary I2C are logically driven by the primary I2C bus
    {0x23, 0x00}, // FIFO Enable.disenable  
    {0x19, 0x63}, // Sample Rate Divider.Sample Rate = 1KHz / (1 + 99)  
    {0x1A, 0x13}, // EXT_SYNC_SET = GYRO_XOUT_L[0]; Bandwidth = 3
    {0x6B, 0x01}, // Power Management 1.PLL with X axis gyroscope reference
    {0x6C, 0x00}  // Power Management 2
};

static esp_err_t  i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *reg_addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
* @brief 初始化 mpu6050
*/
esp_err_t mpu6050_init()
{
    esp_err_t esp_err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU6050_I2C_SCL,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ,  // select frequency specific to your project
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err = i2c_param_config(MPU6050_I2C_PORT_NUM, &conf);
    printf("i2c_param_config: %d \n", esp_err);

    esp_err = i2c_driver_install(MPU6050_I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
    printf("i2c_driver_install: %d \n", esp_err);

    for (size_t i = 0; i < 11; i++)
    {
        esp_err = i2c_master_write_slave(MPU6050_I2C_PORT_NUM, mpu6050_init_cmd[i], 2);
        if (i == 0)
            vTaskDelay(500 / portTICK_RATE_MS);
    }
    printf("mpu6050_init_cmd: %d \n", esp_err);
    return esp_err;
}

/**
* @brief 读取加速度计、温度和陀螺仪数据
*/
measurement_out_t mpu6050_get_value()
{
    uint8_t *measurement_bytes_out = (uint8_t *)malloc(14);
    i2c_master_read_slave(MPU6050_I2C_PORT_NUM, 0x3B, measurement_bytes_out, 14);
    measurement_out_t measurement_out = {
        .accel_out.accel_xout = (int16_t)(measurement_bytes_out[0]<<8 | measurement_bytes_out[1]),
        .accel_out.accel_yout = (int16_t)(measurement_bytes_out[2]<<8 | measurement_bytes_out[3]),
        .accel_out.accel_zout = (int16_t)(measurement_bytes_out[4]<<8 | measurement_bytes_out[5]),
        .temp_out.temp_xout = (int16_t)(measurement_bytes_out[6]<<8 | measurement_bytes_out[7]),
        .gyro_out.gyro_xout = (int16_t)(measurement_bytes_out[8]<<8 | measurement_bytes_out[9]),
        .gyro_out.gyro_yout = (int16_t)(measurement_bytes_out[10]<<8 | measurement_bytes_out[11]),
        .gyro_out.gyro_zout = (int16_t)(measurement_bytes_out[12]<<8 | measurement_bytes_out[13]),
    };
    return measurement_out;
}