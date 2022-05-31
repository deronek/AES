#include "esp_port.h"

esp_err_t esp_i2c_write(unsigned char device_address, unsigned char reg_addr, unsigned char write_size, unsigned char const *write_buffer)
{
    // // create buffer with reg_addr before the rest of the data
    // // TODO: can this be optimized? creating copy of the whole array is not that great
    // unsigned char new_len = length + 1;
    // unsigned char write_buf[new_len];
    // write_buf[0] = reg_addr;
    // memcpy(write_buf + 1, data, length * sizeof(*data));

    // return i2c_master_write_to_device(I2C_MASTER_NUM, slave_addr, write_buf, new_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_LINK_RECOMMENDED_SIZE(1)] = {0};

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert(handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK)
    {
        goto end;
    }

    err = i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
    if (err != ESP_OK)
    {
        goto end;
    }

    err = i2c_master_write_byte(handle, reg_addr, true);
    if (err != ESP_OK)
    {
        goto end;
    }

    err = i2c_master_write(handle, write_buffer, write_size, true);
    if (err != ESP_OK)
    {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

end:
    i2c_cmd_link_delete_static(handle);
    return err;
}