/* Example for using SSD1608 Eink/EPaperDisplay (EPD)
 */

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern "C" {
    void app_main(void);
}

static const char* TAG = "SSD1680";

#define PIN_NUM_BUSY 5
#define PIN_NUM_RES 18
#define PIN_NUM_DC 21
#define PIN_NUM_CS 22
#define PIN_NUM_SCK 19  // CLK
#define PIN_NUM_SDI 23  // MOSI


#define WIDTH 250
#define HEIGHT 122

inline gpio_num_t int_to_pin(int pin) {
  assert(0 <= pin);
  assert(pin < GPIO_NUM_MAX);
  return static_cast<gpio_num_t>(pin);
}

class SSD1680Eink {
  enum Command : uint8_t {
    DriverOutputControl = 0x01,
    GateDrivingVoltageControl = 0x03,
    SourceDrivingVoltageControl = 0x04,
    InitialCodeSettingOTPProgram = 0x08,
    WriteRegisterForInitialCodeSetting = 0x09,
    ReadRegisterForInitialCodeSetting = 0x0A,
    BoosterSoftStartControl = 0x0C,
    DeepSleepMode = 0x10,
    DataEntryModeSetting = 0x11,
    SoftwareReset = 0x12,
    HVReadyDetection = 0x14,
    VCIDetection = 0x15,
    TemperatureSensorControl = 0x18,
    // ...
    MasterActivation = 0x20,
    DisplayUpdateControl1 = 0x21,
    DisplayUpdateControl2 = 0x22,
    WriteRAMBlackWhite = 0x24,
    WriteRAMRed = 0x26,
    // ...
    WriteVCOMRegister = 0x2C,
    // ...
    BorderWaveformControl = 0x3C,
    // ...
    SetRAMXAddressPositions = 0x44,
    SetRAMYAddressPositions = 0x45,
    SetRAMXAddressCounter = 0x4E,
    SetRAMYAddressCounter = 0x4F,
    NOP = 0x7F,
  };

  struct cmds {
    Command cmd;
    uint8_t data[4];
    uint8_t len;
  };

  spi_device_handle_t spi;

  unsigned width, height;
  gpio_num_t dc_pin, res_pin, cs_pin, busy_pin;

  size_t buffer_size;
  uint8_t* blackwhite_buffer;
  uint8_t* red_buffer;

  static void spi_pre_transfer_cb(spi_transaction_t *t) {
    // t->user encodes dc level in the lowest bit and dc_pin in the bits above.
    const int dc = reinterpret_cast<int>(t->user) & 1;
    const gpio_num_t dc_pin = int_to_pin(reinterpret_cast<int>(t->user) >> 1);
    gpio_set_level(dc_pin, dc);
  }

  void busy_wait() {
    while (gpio_get_level(busy_pin) == 1) {  // Display is busy on high
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }

  esp_err_t hardware_reset() {
    ESP_RETURN_ON_ERROR(gpio_set_level(res_pin, 1), TAG, "set(res,1)");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_RETURN_ON_ERROR(gpio_set_level(res_pin, 0), TAG, "set(res,0)");  // active on low!
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_RETURN_ON_ERROR(gpio_set_level(res_pin, 1), TAG, "set(res,1)");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return ESP_OK;
  }

  esp_err_t send(const uint8_t* data, size_t len, bool is_data, bool keep_cs_active) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len*8;
    t.tx_buffer = data;
    t.user = reinterpret_cast<void*>(static_cast<unsigned>(is_data) | static_cast<int>(dc_pin) << 1);
    if (keep_cs_active) {  // TODO: remove?
      t.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    }
    return spi_device_polling_transmit(spi, &t);
  }

  esp_err_t send_command(uint8_t cmd, bool keep_cs_active) {
    return send(&cmd, 1, false, keep_cs_active);
  }

  esp_err_t send_commands(const cmds* c, size_t l) {
    for (size_t i = 0; i < l; ++i) {
      ESP_RETURN_ON_ERROR(send_command(c[i].cmd, false),
                          TAG, "send_command (%u)", i);
      ESP_RETURN_ON_ERROR(send(c[i].data, c[i].len, true, false),
                          TAG, "send data (%u)", i);
    }
    return ESP_OK;
  }

  const uint8_t BLACK = 0;
  const uint8_t WHITE = 1;
  const uint8_t NOT_RED = 1;
  const uint8_t RED = 1;

public:
  explicit SSD1680Eink(unsigned width, unsigned height, gpio_num_t dc_pin, gpio_num_t res_pin, gpio_num_t cs_pin, gpio_num_t busy_pin)
    : spi{nullptr}, width{width}, height{height}, dc_pin{dc_pin}, res_pin{res_pin}, cs_pin{cs_pin}, busy_pin{busy_pin}, buffer_size{0}, blackwhite_buffer{nullptr}, red_buffer{nullptr}
  {
  }

  SSD1680Eink(const SSD1680Eink&) = delete;
  SSD1680Eink& operator=(const SSD1680Eink&) = delete;

  ~SSD1680Eink() {
    free(blackwhite_buffer);
    free(red_buffer);
  }

  void clear_buffer() {
    if (buffer_size > 0) {
      memset(blackwhite_buffer, WHITE, buffer_size);
      memset(red_buffer, NOT_RED, buffer_size);
    }
  }

  enum Pixel {
    White = 1,
    Black = 0,
    Red = 2
  };

  void draw_pixel(unsigned x, unsigned y, Pixel p) {
    const unsigned n = x + y/8 * width;
    assert(n < buffer_size);
    switch (p) {
    case Pixel::White:
      blackwhite_buffer[n] |= 1 << (y & 7);  // White = 1
      red_buffer[n] &= ~(1 << (y & 7));  // Clear red
      break;
    case Pixel::Black:
      blackwhite_buffer[n] &= ~(1 << (y & 7));
      red_buffer[n] &= ~(1 << (y & 7));
      break;
    case Pixel::Red:
      red_buffer[n] |= 1 << (y & 7);
      break;
    }
  }

  esp_err_t init() {
    // Height in bytes. Add extra byte if we need more bits.
    const uint8_t height_bytes = height/8 + (height%8 != 0);

    if (buffer_size == 0) {
      buffer_size = width * height_bytes;
    }
    if (!blackwhite_buffer) {
      blackwhite_buffer = (uint8_t*)malloc(buffer_size);
      if (!blackwhite_buffer) {
        return ESP_ERR_NO_MEM;
      }
    }
    if (!red_buffer) {
      red_buffer = (uint8_t*)malloc(buffer_size);
      if (!red_buffer) {
        return ESP_ERR_NO_MEM;
      }
    }
    clear_buffer();

    ESP_RETURN_ON_ERROR(gpio_set_direction(dc_pin, GPIO_MODE_OUTPUT),
                        TAG, "gpio_set_direction(dc_pin=%d, out)", dc_pin);
    ESP_RETURN_ON_ERROR(gpio_set_direction(res_pin, GPIO_MODE_OUTPUT),
                        TAG, "gpio_set_direction(res_pin=%d, out)", res_pin);
    ESP_RETURN_ON_ERROR(gpio_set_direction(busy_pin, GPIO_MODE_INPUT),
                        TAG, "gpio_set_direction(busy_pin=%d, in)", busy_pin);

    // Set Initial Configuration
    spi_device_interface_config_t devcfg={
      .mode=0,
      .clock_speed_hz=10*1000*1000, // hmm? 40 MHz
      .spics_io_num=cs_pin,
      .queue_size=7,
      .pre_cb=&SSD1680Eink::spi_pre_transfer_cb,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(HSPI_HOST, &devcfg, &spi), TAG, "spi_bus_add_device");

    ESP_RETURN_ON_ERROR(hardware_reset(), TAG, "HardwareReset");
    ESP_RETURN_ON_ERROR(send_command(Command::SoftwareReset, false), TAG, "SoftwareReset");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Send Initialization Code
    const uint8_t endy_lo = width - 1;
    const uint8_t endy_hi = (width - 1) >> 8;

    const cmds init_cmds[] = {
      { Command::DataEntryModeSetting, {0x03}, 1 },  // y,x increment, x-mode
      { Command::BorderWaveformControl, {0x05}, 1},  // Follow LUT, LUT1
      { Command::WriteVCOMRegister, {0x36}, 1}, //  36???
      { Command::GateDrivingVoltageControl, {0x17}, 1},  // VGH, 20V
      { Command::SourceDrivingVoltageControl, {0x41, 0x00, 0x32}, 3},  // 
      { Command::SetRAMXAddressCounter, {0x01}, 1},
      { Command::SetRAMYAddressCounter, {0x00, 0x00}, 2},
      { Command::SetRAMXAddressPositions, {0x01, height_bytes}, 2},  // 1 to height/8
      { Command::SetRAMYAddressPositions, {0x00, 0x00, endy_lo, endy_hi}, 4},  // 0 to width
      { Command::DriverOutputControl, {endy_lo, endy_hi, 0x00}, 3},
    };
    ESP_RETURN_ON_ERROR(send_commands(init_cmds, sizeof(init_cmds)/sizeof(init_cmds[0])),
                        TAG, "send_commands");

    

    return ESP_OK;
  }

  esp_err_t display() {
    // SetRAMX/YAddressCounter
    const cmds setramcmds[] = {
      { Command::SetRAMXAddressCounter, {0x01}, 1},
      { Command::SetRAMYAddressCounter, {0x00, 0x00}, 2},
    };
    ESP_RETURN_ON_ERROR(send_commands(setramcmds, sizeof(setramcmds)/sizeof(setramcmds[0])),
                        TAG, "SetRAMX/YAddressCounter");

    // write ram
    ESP_RETURN_ON_ERROR(send_command(Command::WriteRAMBlackWhite, false), TAG, "RAM BW Cmd");
    ESP_RETURN_ON_ERROR(send(blackwhite_buffer, buffer_size, true, false), TAG, "RAM BW");
    ESP_RETURN_ON_ERROR(send_command(Command::WriteRAMRed, false), TAG, "RAM Red Cmd");
    ESP_RETURN_ON_ERROR(send(red_buffer, buffer_size, true, false), TAG, "RAM Red");

    // update
    const cmds dspctrl[] = { {Command::DisplayUpdateControl2, {0xF4}, 1} };  // ?? 0xF4?
    ESP_RETURN_ON_ERROR(send_commands(dspctrl, sizeof(dspctrl)/sizeof(dspctrl[0])), TAG, "displaycontrol2");
    ESP_RETURN_ON_ERROR(send_command(Command::MasterActivation, false), TAG, "masteractivate");
    busy_wait();

    return ESP_OK;
  }

  esp_err_t sleep() {
    return send_command(Command::DeepSleepMode, false);
  }
};

void app_main(void) {
  spi_bus_config_t buscfg={
    .mosi_io_num=PIN_NUM_SDI,
    .miso_io_num=-1,
    .sclk_io_num=PIN_NUM_SCK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=16*320*2+8, // TODO?!
  };
  ESP_LOGI(TAG, "Init SPI!");
  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ESP_LOGI(TAG, "Create display!");
  SSD1680Eink ssd1680eink(WIDTH, HEIGHT, int_to_pin(PIN_NUM_DC), int_to_pin(PIN_NUM_RES), int_to_pin(PIN_NUM_CS), int_to_pin(PIN_NUM_BUSY));
  ESP_LOGI(TAG, "Init display!");
  ESP_ERROR_CHECK(ssd1680eink.init());

  ESP_LOGI(TAG, "Draw!");
  for (unsigned x = 0; x < 20; ++x) {
    for (unsigned y = 0; y < 20; ++y) {
      const SSD1680Eink::Pixel p = (5 < x && x < 10 && 5 < y && y < 10) ? SSD1680Eink::Red : SSD1680Eink::Black;
      ssd1680eink.draw_pixel(x, y, p);
    }
  }

  ESP_LOGI(TAG, "Display!");
  ESP_ERROR_CHECK(ssd1680eink.display());
  ESP_LOGI(TAG, "Wait 10s!");
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "Sleep!");
  ESP_ERROR_CHECK(ssd1680eink.sleep());
  ESP_LOGI(TAG, "Bye!");
}
