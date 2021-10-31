/* SSD1306 code for ESP32.

   Datasheet https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "font5x5.hpp"
#include "font10x10.hpp"

extern "C" {
    void app_main(void);
}

static const char* TAG = "SSD1306";


#define I2C_MASTER_SCL_IO           22 /*CONFIG_I2C_MASTER_SCL*/      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21 /*CONFIG_I2C_MASTER_SDA*/      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define WIDTH 128
#define HEIGHT 68

constexpr size_t buffer_size(size_t width, size_t height) {
  return width * ((height + 7) / 8);
}

class SSD1306Display {
  enum Command : uint8_t {
    // Fundamental Command Table
    SetContrast = 0x81,  // +1 byte: Contrast value (the higher the more contrast)
    SetEntireDisplayOn = 0xA4, // Lower bit: 0 - Resume, 1 - Ignore RAM content
    SetNormalDisplay = 0xA6,
    SetInverseDisplay = 0xA7,
    SetDisplayOFF = 0xAE,
    SetDisplayON = 0xAF,

    // Scrolling Command Table
    DeactivateScroll = 0x2E,

    // Addressing Command Table
    SetMemoryAddressingMode = 0x20,  // +1 byte: lower 2 bits: 0b00 Horizontal, 0b01 Vertical, 0b10 Page
    SetColumnAddress = 0x21, // + 2 byte
    SetPageAddress = 0x22,  // +2 byte

    // Hardware Configuration Command Table
    SetDisplayStartLine = 0x40,  // Lower 5 bits are startline
    SetSegmentRemap = 0xA0,  // Lower bit: 0 - col 0 is mapped to SEG0, 1 - col 127 is mapped to SEG0
    SetMultiplexRatio = 0xA8,  // +1 byte
    SetCOMOutputScanDirection = 0xC0,  // 3rd Bit: 0 - normal mode, 1 - remapped mode
    SetDisplayOffset = 0xD3,  // +1 byte (5 bits for offset)
    SetCOMPinsHardwareConfig = 0xDA,  // +1 byte, 

    // Timing and Driving Command Table
    SetDisplayClock = 0xD5,  // +1 byte for clock
    SetPrechargePeriod = 0xD9,  // +1 byte,
    ChargePumpSetting = 0x8D,  // +1 byte, either 0x14 (Enable), 0x10 (Disable)
    SetVCOMDeselectLevel = 0xDB,  // + 1 byte
    NOP = 0xE3,
  };

  uint8_t addr;
  i2c_port_t i2c_num;
  uint8_t width, height;
  uint8_t *buffer;

  /**
   * @brief i2c_write function.
   *
   * @param data byte data
   * @param n length of data
   * @param initial_byte if this is different than 0xFF send it before writing data
   * @param ack_en wait for ack
   */
  esp_err_t i2c_write(const uint8_t* data, size_t n, uint8_t initial_byte = 0xFF, bool ack_en = true) {
    esp_err_t err = ESP_OK;
    uint8_t buf[I2C_LINK_RECOMMENDED_SIZE(5)] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buf, sizeof(buf));
    assert (handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_master_start(..) -> %d", err);
      goto end;
    }

    err = i2c_master_write_byte(handle, addr << 1 | I2C_MASTER_WRITE, ack_en);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_master_write_byte(..., 'addr', ...) -> %d", err);
      goto end;
    }

    if (initial_byte != 0xFF) {
      err = i2c_master_write_byte(handle, initial_byte, ack_en);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_byte(..., initial_byte, ...) -> %d", err);
        goto end;
      }
    }

    err = i2c_master_write(handle, data, n, ack_en);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_master_write(...) -> %d", err);
      goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(i2c_num, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2c_master_cmd_begin(...) -> %d", err);
    }

  end:
    i2c_cmd_link_delete_static(handle);
    ESP_LOGI(TAG, "i2c_write(%p, %u, %x) = %d", data, n, initial_byte, err);
    return err;
  }

public:
  explicit SSD1306Display(uint8_t width, uint8_t height, i2c_port_t i2c_num, uint8_t addr = 0)
    : addr{addr}, i2c_num{i2c_num}, width{width}, height{height}, buffer{nullptr}
  {
    if (addr == 0) {
      // height == 64 can also be 0x3C if SA0 is grounded
      addr = height == 32 ? 0x3C : 0x3D;
      ESP_LOGI(TAG, "Setting addr = %X", addr);
    }
  }

  ~SSD1306Display() {
    free(buffer);
  }

  void clear_buffer() {
    if (!buffer) {
      return;
    }
    memset(buffer, 0, buffer_size(width, height));
  }
  uint8_t* get_buffer() {
    return buffer;
  }
  const uint8_t* get_buffer() const {
    return buffer;
  }
  void clear_pixel(uint8_t x, uint8_t y) {
    buffer[x + y/8 * width] &= ~(1 << (y & 7));
  }
  void set_pixel(uint8_t x, uint8_t y) {
    buffer[x + y/8 * width] |= 1 << (y & 7);
  }
  void invert_pixel(uint8_t x, uint8_t y) {
    buffer[x + y/8 * width] ^= 1 << (y & 7);
  }
  void draw_pixel(uint8_t x, uint8_t y, uint8_t c) {
    if (c) {
      set_pixel(x, y);
    } else {
      clear_pixel(x, y);
    }
  }

  uint8_t get_width() const { return width; }

  /**
   * @brief copy rectangle from src to buffer.
   *
   * @param x The x location to start copying to
   * @param y The y location to start copying to
   * @param w The width of the rectangle
   * @param w The height of the rectangle
   * @param src The src buffer
   * @param swidth The width of the src buffer
   * @param src_x X position to start copying from (default 0)
   * @param src_y Y position to start copying from (default 0)
   */
  void blit(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t* src, size_t swidth, uint8_t src_x=0, uint8_t src_y=0) {
    assert(x + w <= width);
    assert(src_x + w <= swidth);
    const uint8_t begin = y / 8;
    const uint8_t begin_r = y % 8;
    const uint8_t end = (y+h) / 8;
    const uint8_t end_r = (y+h) % 8;

    if (begin == end) {
      const uint8_t mbegin = (0xFF << begin_r) & 0xFF;
      const uint8_t mend = 0xFF >> (8 - end_r);
      const uint8_t mask = mbegin & mend;
      for (size_t i = 0; i < w; ++i) {
        const size_t n = x + i + begin * width;
        const size_t m = src_x + i + src_y/8 * swidth;
        buffer[n] |= mask & (src[m] << begin_r);
      }
    } else {
      uint8_t mask = (0xFF << begin_r) & 0xFF;
      for (size_t i = 0; i < w; ++i) {
        const size_t n = x + i + begin * width;
        const size_t m = src_x + i + src_y/8 * swidth;
        buffer[n] |= mask & (src[m] << begin_r);
      }

      for (size_t j = 1; j < end-begin; ++j) {
        for (size_t i = 0; i < w; ++i) {
          const size_t n = x + i + (begin + j) * width;
          const size_t m = src_x + i + (src_y/8 + j) * swidth;
          buffer[n] = src[m];
        }
      }

      mask = 0xFF >> (8 - end_r);
      for (size_t i = 0; i < w; ++i) {
        const size_t n = x + i + end * width;
        const size_t m = src_x + i + (src_y + h)/8 * swidth;
        buffer[n] |= mask & src[m];
      }
    }
  }


  /**
   * @brief Initialise display
   *
   * @param contrast
   *        The higher the more contrast.
   * @param mux
   *        Set multiplex to mux+1, valid range 15 to 63 and 0xFF,
   *        which is turned into height-1.
   * @param offset
   *        Vertical shift, valid range 0 to 63.  (Table 10-1)
   * @param startline
   *        Display start line. (Table 10-1)
   * @param externalvcc
   *        Set to false to generate display voltage from 3.3V source.
   */
  esp_err_t init(uint8_t contrast = 0xCF /* 7F? */, uint8_t mux=0xFF, uint8_t offset=0x00, uint8_t startline=0x00, bool externalvcc=false) {
    if (!buffer) {
      buffer = (uint8_t*)malloc(buffer_size(width, height));
      if (!buffer) {
        return ESP_ERR_NO_MEM;
      }
    }
    clear_buffer();

    if (mux == 0xFF) {
      mux = height - 1;
    }
    assert(15 <= mux && mux <= 63);
    assert(offset <= 63);
    assert(startline <= 63);
    uint8_t commands[] = {
      Command::SetDisplayOFF,
      Command::SetDisplayClock,
      0x80,  // Fosc = 8, Div = 0
      
      Command::SetMultiplexRatio,
      mux,
      Command::SetDisplayOffset,
      offset,
      uint8_t(Command::SetDisplayStartLine | startline),

      Command::ChargePumpSetting,
      uint8_t(externalvcc ? 0x10 : 0x14),

      Command::SetMemoryAddressingMode,
      0x00,  // Horizontal Addressing Mode

      Command::SetSegmentRemap | 0x01,
      Command::SetCOMOutputScanDirection | 0x8,  // remapped mode
      Command::SetCOMPinsHardwareConfig,
      0x12,  // TODO!

      Command::SetContrast,
      contrast,

      Command::SetPrechargePeriod,
      uint8_t(externalvcc ? 0x22 : 0xF1),

      Command::SetVCOMDeselectLevel,
      0x40,   // TODO: 0x00?!


      Command::SetEntireDisplayOn | 0x00,  // Resume
      Command::SetNormalDisplay,
      Command::DeactivateScroll,
      Command::SetDisplayON,
    };
    return i2c_write(commands, sizeof(commands)/sizeof(commands[0]), 0x00);
  }

  esp_err_t set_contrast(uint8_t contrast) {
    uint8_t commands[] = {
      Command::SetContrast,
      contrast,
    };
    return i2c_write(commands, sizeof(commands)/sizeof(commands[0]), 0x00);
  }

  esp_err_t invert_display(bool inv) {
    uint8_t cmd = inv ? Command::SetInverseDisplay : Command::SetNormalDisplay;
    return i2c_write(&cmd, 1, 0x00);
  }

  /// Display current buffer on screen
  esp_err_t display() {
    uint8_t commands[] = {
      Command::SetPageAddress,
      0x00,
      0xFF,
      Command::SetColumnAddress,
      0x00,
      uint8_t(width-1),
    };
    esp_err_t err = i2c_write(commands, sizeof(commands)/sizeof(commands[0]), 0x00);
    if (err != ESP_OK)
      return err;
    return i2c_write(buffer, 1 + width * ((height + 7) / 8), 0x40);
  }

  /// TODO: Scrolling
};

const size_t INVALID_GLYPH_IDX = 0xFFFF;
size_t char_to_glyphidx(char c) {
  if ('0' <= c && c <= '9') {
    return c - '0';
  } else if ('a' <= c && c <= 'z') {
    return c - 'a' + 10;
  } else if ('A' <= c && c <= 'Z') {
    return c - 'A' + 10;
  } else if (c == ' ') {
    return 'z' - 'a' + 11;
  } else if (c == '!') {
    return 'z' - 'a' + 12;
  } else if (c == '?') {
    return 'z' - 'a' + 13;
  } else if (c == ':') {
    return 'z' - 'a' + 14;
  } else if (c == ';') {
    return 'z' - 'a' + 15;
  } else if (c == '.') {
    return 'z' - 'a' + 16;
  } else if (c == ',') {
    return 'z' - 'a' + 17;
  } else if (c == '-') {
    return 'z' - 'a' + 18;
  } else if (c == '+') {
    return 'z' - 'a' + 19;
  } else if (c == '%') {
    return 'z' - 'a' + 20;
  }

  ESP_LOGE(TAG, "Unsupported character '%c'", c);
  return INVALID_GLYPH_IDX;  // Default to space
}

esp_err_t render_font(SSD1306Display &display, const font& font, const char *s, size_t n, uint8_t x, uint8_t y, uint8_t dx=1) {
  uint8_t x_ = x;
  for (size_t i = 0; i < n; ++i) {
    const size_t glyphidx{char_to_glyphidx(s[i])};
    if (glyphidx == INVALID_GLYPH_IDX) {  // Skip invalid glyphs
      continue;
    }
    display.blit(x_, y, font.width, font.height, font.glyphs[glyphidx], font.width);
    x_ += font.width + dx;
    if (x_ + font.width >= display.get_width()) {
      return ESP_ERR_INVALID_SIZE;
    }
  }

  return ESP_OK;
}

void app_main(void) {
  const int i2c_master_port = I2C_MASTER_NUM;

  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

  i2c_param_config(i2c_master_port, &conf);

  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
  ESP_LOGI(TAG, "I2C Initialized");

  SSD1306Display display(128, 64, i2c_master_port, 0x3C);
  ESP_LOGI(TAG, "Display created");
  ESP_ERROR_CHECK(display.init());
  ESP_LOGI(TAG, "Display initialized");
  // for(uint8_t x = 30; x <= 98; ++x) {
  //   for(uint8_t y = 10; y <= 56; ++y) {
  //     display.set_pixel(x, y);
  //   }
  // }

#if 0
  for (uint8_t x = 0; x <= 128; ++x) {
    // Top 15 pixels are yellow, rest is blue.
    for(uint8_t y = 0; y <= 15; ++y) {
      display.set_pixel(x, y);
    }
  }
  ESP_LOGI(TAG, "Display pixels set");
  ESP_ERROR_CHECK(display.display());
  ESP_LOGI(TAG, "Display displayed");

  for (int i = 0; i < 10; ++i) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  display.invert_display(true);
  ESP_LOGI(TAG, "Display inverted");

  for (int i = 0; i < 10; ++i) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  display.invert_display(false);
  ESP_LOGI(TAG, "Display normal");

  uint8_t contrast = 0;
  for (int i = 0; i < 20; ++i) {
    display.set_contrast(contrast);
    ESP_LOGI(TAG, "Display contrast %x", contrast);
    contrast += 10;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
#else
  // const uint8_t src[] = {0b11111110, 0b11111110, 0b01100000, 0b01100000, 0b01100000, 0b01100000, 0b01100000, 0b01100000, 0b11111110, 0b11111110, 0b00000000, 0b11111110, 0b11111110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111000, 0b11111000, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b11111000, 0b11111000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000000, 0b00000000, 0b11100000, 0b11100000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000000, 0b11111000, 0b11111000, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b11111000, 0b11111000, 0b00000000, 0b11111110, 0b11111110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b10011000, 0b10011000, 0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111110, 0b11111110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b11111000, 0b11111000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01111110, 0b01111110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000000, 0b00000111, 0b00000111, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b00000001, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000001, 0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000001, 0b00000001, 0b00000110, 0b00000110, 0b00000001, 0b00000001, 0b00000110, 0b00000110, 0b00000001, 0b00000001, 0b00000000, 0b00000001, 0b00000001, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000001, 0b00000001, 0b00000000, 0b00000111, 0b00000111, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000111, 0b00000111, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000110, 0b00000001, 0b00000001, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000110, 0b00000110, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};
  // const uint8_t swidth = 132;
  // display.blit(0, 32, 128, 10, src, swidth);


  render_font(display, font5x5::font5x5, "Hello World!", 12, 0, 32);
  render_font(display, font10x10::font10x10, "Hello World!", 12, 0, 44);
  ESP_LOGI(TAG, "Display pixels set");
  ESP_ERROR_CHECK(display.display());
  ESP_LOGI(TAG, "Display displayed");
  for (int i = 0; i < 10; ++i) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
#endif

  ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
  ESP_LOGI(TAG, "I2C unitialized successfully");
}
