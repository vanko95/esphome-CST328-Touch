#include "cst328_touchscreen.h"

namespace esphome
{
  namespace cst328
  {

#define HYN_REG_MUT_DEBUG_INFO_MODE 0xD101
#define HYN_REG_MUT_RESET_MODE 0xD102
#define HYN_REG_MUT_DEBUG_RECALIBRATION_MODE 0xD104
#define HYN_REG_MUT_DEEP_SLEEP_MODE 0xD105
#define HYN_REG_MUT_DEBUG_POINT_MODE 0xD108
#define HYN_REG_MUT_NORMAL_MODE 0xD109

/****************HYN_REG_MUT_DEBUG_INFO_MODE address start***********/
#define HYN_REG_MUT_DEBUG_INFO_IC_CHECKSUM 0xD208
#define HYN_REG_MUT_DEBUG_INFO_FW_VERSION 0xD204
#define HYN_REG_MUT_DEBUG_INFO_IC_TYPE 0xD202
#define HYN_REG_MUT_DEBUG_INFO_PROJECT_ID 0xD200
#define HYN_REG_MUT_DEBUG_INFO_BOOT_TIME 0xD1FC
#define HYN_REG_MUT_DEBUG_INFO_RES_Y 0xD1FA
#define HYN_REG_MUT_DEBUG_INFO_RES_X 0xD1F8
#define HYN_REG_MUT_DEBUG_INFO_KEY_NUM 0xD1F7
#define HYN_REG_MUT_DEBUG_INFO_TP_NRX 0xD1F6
#define HYN_REG_MUT_DEBUG_INFO_TP_NTX 0xD1F4

#define CST328_LCD_TOUCH_MAX_POINTS 5
/* CST328 registers */
#define ESP_LCD_TOUCH_CST328_READ_Number_REG 0xD005
#define ESP_LCD_TOUCH_CST328_READ_XY_REG 0xD000
#define ESP_LCD_TOUCH_CST328_READ_Checksum_REG 0x80FF
#define ESP_LCD_TOUCH_CST328_CONFIG_REG 0x8047

    void CST328Touchscreen::setup()
    {
      esph_log_config(TAG, "Setting up CST328 Touchscreen...");
      if (this->reset_pin_ != nullptr)
      {
        this->reset_pin_->setup();
        this->reset_pin_->digital_write(true);
        delay(50);
        this->reset_pin_->digital_write(false);
        delay(50);
        this->reset_pin_->digital_write(true);
        delay(50);
        this->set_timeout(30, [this]
                          { this->continue_setup_(); });
      }
      else
      {
        this->continue_setup_();
      }
    }

    void CST328Touchscreen::update_touches()
    {
      esph_log_v(TAG, "update touches");

      uint8_t buf[41];
      uint8_t touch_cnt = 0;
      uint8_t clear = 0;
      uint8_t Over = 0xAB;
      size_t i = 0, num = 0;

      this->read_bytes(ESP_LCD_TOUCH_CST328_READ_Number_REG, buf, 1);
      if ((buf[0] & 0x0F) == 0x00)
      {
        this->write_register16(ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1); // No touch data
      }
      else
      {
        /* Count of touched points */
        touch_cnt = buf[0] & 0x0F;
        if (touch_cnt > CST328_LCD_TOUCH_MAX_POINTS || touch_cnt == 0)
        {
          this->write_register16(ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
          return;
        }
        /* Read all points */
        this->read_bytes(ESP_LCD_TOUCH_CST328_READ_XY_REG, &buf[1], 27);
        /* Clear all */
        this->write_register16(ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
        esph_log_v(TAG, " points=%d ", touch_cnt);

        /* Number of touched points */
        if (touch_cnt > CST328_LCD_TOUCH_MAX_POINTS)
          touch_cnt = CST328_LCD_TOUCH_MAX_POINTS;
        /* Fill all coordinates */
        for (i = 0; i < touch_cnt; i++)
        {
          if (i > 0)
            num = 2;
          uint8_t id = buf[(i * 5) + num] >> 4;
          int16_t x = (uint16_t)(((uint16_t)buf[(i * 5) + 2 + num] << 4) + ((buf[(i * 5) + 4 + num] & 0xF0) >> 4));
          int16_t y = (uint16_t)(((uint16_t)buf[(i * 5) + 3 + num] << 4) + (buf[(i * 5) + 4 + num] & 0x0F));
          int16_t z = ((uint16_t)buf[(i * 5) + 5 + num]);

          this->add_raw_touch_position_(id, x, y, z);
          esph_log_v(TAG, "Read touch %d: %d/%d", id, x, y);
        }
      }
    }

    void CST328Touchscreen::continue_setup_()
    {
      uint8_t buffer[8];
      if (this->interrupt_pin_ != nullptr)
      {
        this->interrupt_pin_->setup();
        this->attach_interrupt_(this->interrupt_pin_, gpio::INTERRUPT_FALLING_EDGE);
      }
      buffer[0] = 0x01;

      if (this->write_register16(HYN_REG_MUT_DEBUG_INFO_MODE, buffer, 0) != i2c::ERROR_OK)
      {
        esph_log_e(TAG, "Write byte to HYN_REG_MUT_DEBUG_INFO_MODE failed");
        this->mark_failed();
        return;
      }
      delay(10);
      if (this->read16_(HYN_REG_MUT_DEBUG_INFO_FW_VERSION, buffer, 4))
      {
        uint16_t chip_id = buffer[2] + (buffer[3] << 8);
        uint16_t project_id = buffer[0] + (buffer[1] << 8);
        esph_log_config(TAG, "Chip ID %X, project ID %x", chip_id, project_id);
      }
      if (this->x_raw_max_ == 0 || this->y_raw_max_ == 0)
      {
        if (this->read16_(HYN_REG_MUT_DEBUG_INFO_RES_X, buffer, 4))
        {
          this->x_raw_max_ = buffer[0] + (buffer[1] << 8);
          this->y_raw_max_ = buffer[2] + (buffer[3] << 8);
        }
        else
        {
          this->x_raw_max_ = this->display_->get_native_width();
          this->y_raw_max_ = this->display_->get_native_height();
        }
      }
      uint8_t buf[24];
      if (this->read16_(0xD1F4, buf, 24))
      {
        esph_log_config(TAG, "D1F4:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[0], buf[1], buf[2], buf[3]);
        esph_log_config(TAG, "D1F8:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[4], buf[5], buf[6], buf[7]);
        esph_log_config(TAG, "D1FC:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[8], buf[9], buf[10], buf[11]);
        esph_log_config(TAG, "D200:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[12], buf[13], buf[14], buf[15]);
        esph_log_config(TAG, "D204:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[16], buf[17], buf[18], buf[19]);
        esph_log_config(TAG, "D208:0x%02x,0x%02x,0x%02x,0x%02x\r\n", buf[20], buf[21], buf[22], buf[23]);
        esph_log_config(TAG, "CACA Read:0x%04x\r\n", (((uint16_t)buf[11] << 8) | buf[10]));
      }

      if (this->write_register16(HYN_REG_MUT_NORMAL_MODE, buffer, 0) != i2c::ERROR_OK)
      {
        esph_log_e(TAG, "Write byte to HYN_REG_MUT_NORMAL_MODE failed");
        this->mark_failed();
        return;
      }
      this->setup_complete_ = true;
      esph_log_config(TAG, "CST328 Touchscreen setup complete");
    }

    void CST328Touchscreen::dump_config()
    {
      ESP_LOGCONFIG(TAG, "CST328 Touchscreen:");
      LOG_I2C_DEVICE(this);
      LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
      LOG_PIN("  Reset Pin: ", this->reset_pin_);
    }

  } // namespace cst328
} // namespace esphome
