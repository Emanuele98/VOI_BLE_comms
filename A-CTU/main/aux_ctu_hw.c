
#include "driver/gpio.h"
#include "include/aux_ctu_hw.h"

/* 
 * All local readings (on PTU) are not calibrated. Therefore, you must provide a
 * way to convert digital input data to the respective reading types. For example,
 * in the `read_temperature` function, you could convert raw digital data to real
 * temperature by using an intercept and a slope.
 *  
 * It is best to define such values as preprocessor definitions using the #define
 * directive.
 */

static const char* TAG = "HARDWARE";

/**
 * @brief read sensor data
 *
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
float i2c_read_voltage_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, V_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret!=ESP_OK)
    {
        ESP_LOGW(TAG, "voltage reading problem");
        value = 0;
        goto exit;
    }

    //printf("first byte: %02x\n", byte_1);
    //printf("second byte : %02x\n", byte_2);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.00125 * 3.9412;
    printf("sensor val: %.02f [V]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}

float i2c_read_current_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, A_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret!=ESP_OK)
    {
        ESP_LOGW(TAG, "current reading problem");
        value = 0;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 8 | second_byte) * 0.0000025 / 0.012;
    printf("sensor val: %.02f [A]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}


float i2c_read_temperature_sensor(bool n_temp_sens)
{
    int ret;
    uint8_t first_byte, second_byte;
    float value;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if(n_temp_sens)
    {
        i2c_master_write_byte(cmd, T1_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    } else {
        i2c_master_write_byte(cmd, T2_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    }
    i2c_master_write_byte(cmd, T_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if(n_temp_sens)
    {
        i2c_master_write_byte(cmd, T1_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    } else {
        i2c_master_write_byte(cmd, T2_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    }    
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret!=ESP_OK)
    {
        ESP_LOGW(TAG, "temperature reading problem");
        value = 0;
        goto exit;
    }

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    value = (int16_t)(first_byte << 4 | second_byte >> 4) * 0.0625 ;
    printf("sensor val: %.02f [°C]\n", value);

    exit:
        xSemaphoreGive(i2c_sem);
        return value;
}

void enable_full_power_output(void)
{
    enabled = 1;
    gpio_set_level(FULL_POWER_OUT_PIN, 1);
    ESP_LOGI(TAG, "SWITCHIN FULL POWER ON!");
}

void disable_full_power_output(void)
{
    enabled = 0;
    gpio_set_level(FULL_POWER_OUT_PIN, 0);
    ESP_LOGI(TAG, "SWITCHIN FULL POWER OFF!");
}

void enable_low_power_output(void)
{
    enabled = 1;
    gpio_set_level(LOW_POWER_OUT_PIN, 1);
    ESP_LOGI(TAG, "SWITCHIN LOW POWER ON");
}

void disable_low_power_output(void)
{
    enabled = 0;
    gpio_set_level(LOW_POWER_OUT_PIN, 0);
    ESP_LOGI(TAG, "SWITCHIN LOW POWER OFF");
}
