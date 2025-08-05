#include "pins_config.h"
#include "LovyanGFX_Driver.h"

#include <Arduino.h>
#include <lvgl.h>
#include <SPI.h>
#include <Wire.h>

#include <stdbool.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include "ui.h"
#include <log.h>
#include <Preferences.h>
Preferences preferences;
/* Expand IO */
#include <TCA9534.h>
TCA9534 ioex;
  static bool ioex_ready = false;

Stream *uart = &Serial; // Default to Serial
String serialBuffer = "";
LGFX gfx;
/* Change to your screen resolution */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;
bool terminal_screen_active;
const long dimTime = 30000; // 30 seconds
//  Time to dim the screen after last touch
//  If the screen is touched, it will reset the timer
//  If the screen is not touched for this time, it will dim the screen
//  If the screen is touched again, it will reset the timer and brighten the screen
unsigned long lastTouchTime = 0; // Time when the screen was last touched
uint16_t touch_x, touch_y;

//  Display refresh
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (gfx.getStartCount() > 0)
  {
    gfx.endWrite();
  }
  gfx.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::rgb565_t *)&color_p->full);

  lv_disp_flush_ready(disp); //	Tell lvgl that the refresh is complete
}

bool i2cScanForAddress(uint8_t address)
{
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

//  Read touch
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  data->state = LV_INDEV_STATE_REL; // The state of data existence when releasing the finger
  bool touched = gfx.getTouch(&touch_x, &touch_y);
  if (touched)
  {
    //  If the screen is touched, reset the last touch time
    lastTouchTime = millis();
    data->state = LV_INDEV_STATE_PR;

    //  Set coordinates
    data->point.x = touch_x;
    data->point.y = touch_y;
  }
}

void scanI2C()
{
  uart->println("Scanning I2C...");
  for (byte addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0)
    {
      uart->printf("Found device at 0x%02X\n", addr);
    }
  }
}

// Wrapper function for sending I2C commands
void sendI2CCommand(uint8_t command)
{
  uint8_t error;
  // Start sending command to specified address
  Wire.beginTransmission(0x30);
  // Send command
  Wire.write(command);
  // End transmission and return status
  error = Wire.endTransmission();

  if (error == 0)
  {
    uart->print("Command 0x");
    uart->print(command, HEX);
    uart->println(" sent successfully");
  }
  else
  {
    uart->print("Command send error, error code: ");
    uart->println(error);
  }
}
void connectWiFi(lv_event_t *e)
{
  //lv_obj_clear_flag(ui_ConnPopup, LV_OBJ_FLAG_HIDDEN);
  WiFi.mode(WIFI_STA);                                  // Set WiFi mode to Station
  const char *ssid = lv_textarea_get_text(ui_SSID);     // Get SSID from the UI text area
  const char *password = lv_textarea_get_text(ui_PASS); // Get password from the UI text area
  Serial.printf("Connecting to WiFi SSID: %s, Password: %s\n", ssid, password);
  WiFi.begin(ssid, password); // Connect to the WiFi network
  Serial.println("Initializing WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    lv_label_set_text(ui_ConnectionStat, "Connecting...");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    lv_label_set_text(ui_wifistr, "Offline");
    lv_img_set_src(ui_wifistat, &ui_img_1149142576);
  }
  else
  {
    String wifi = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    lv_label_set_text(ui_wifistr, ip.c_str());
    lv_label_set_text(ui_NetConn, wifi.c_str());
    lv_img_set_src(ui_wifistat, &ui_img_wifi_png);
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", password);
    preferences.end();
    Serial.println("Connected to WiFi.");
  }
      lv_obj_add_flag(ui_ConnPopup, LV_OBJ_FLAG_HIDDEN); // Ukryj popup

}
void disconnectWiFi(lv_event_t *e)
{
  // Your code here
}

void setup()

{
    WiFi.mode(WIFI_STA);

  uart = &Serial; // Default to Serial
  static_cast<HardwareSerial *>(uart)->begin(115200);

// Initialize PSRAM and set clock
#if CONFIG_SPIRAM_SUPPORT
  // Force PSRAM clock to 120MHz
  esp_psram_extram_set_clock_rate(120 * 1000000);
  if (!psramInit())
  {
    uart->println("PSRAM initialization failed!");
    while (1)
      ; // Halt to indicate error
  }
  uart->println("PSRAM initialization successful");
#endif

  // Verify PSRAM size
  uart->printf("PSRAM 总大小: %d MB\n", ESP.getPsramSize() / 1024 / 1024);

  // pinMode(19, OUTPUT);//uart1

  // Wire.begin();
  // Wire.end();             // Release current bus
  // Wire.setPins(15, 16);    // Set new pins (SDA=GPIO5, SCL=GPIO6)
  // Wire.begin();           // Reinitialize bus

  Wire.begin(15, 16);
  delay(50);
  while (1)
  {
    if (i2cScanForAddress(0x30) && i2cScanForAddress(0x5D))
    {
      uart->print("The microcontroller is detected: address 0x");
      uart->println(0x30, HEX);
      uart->print("The microcontroller is detected: address 0x");
      uart->println(0x5D, HEX);

      break;
    }
    else
    {
      uart->print("No microcontroller was detected: address 0x");
      uart->println(0x30, HEX);
      uart->print("No microcontroller was detected: address 0x");
      uart->println(0x5D, HEX);
      // Prevent the microcontroller from not starting, adjust backlight
      sendI2CCommand(0x19);
      pinMode(1, OUTPUT);
      digitalWrite(1, LOW);
      // ioex.output(2, TCA9534::Level::L);
      // ioex.output(2, TCA9534::Level::H);
      delay(120);
      pinMode(1, INPUT);

      delay(100);
    }
  }

  if (i2cScanForAddress(0x30) && i2cScanForAddress(0x5D)) // new V1.2
  {
    Wire.beginTransmission(0x30);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.write(0x18);
    Wire.endTransmission();
  }
  else
  {
    ioex.attach(Wire);
    ioex.setDeviceAddress(0x18);
    ioex.config(1, TCA9534::Config::OUT);
    ioex.config(2, TCA9534::Config::OUT);

    /* Turn on backlight*/
    ioex.output(1, TCA9534::Level::H);

    delay(20);
    ioex.output(2, TCA9534::Level::H);
    delay(100);
    pinMode(1, INPUT);
    ioex_ready = true;

    /*end*/
  }

  // Init Display
  gfx.init();
  gfx.initDMA();
  gfx.startWrite();
  gfx.fillScreen(TFT_BLACK);

  lv_init();
  // size_t buffer_size = sizeof(lv_color_t) * LCD_H_RES * LCD_V_RES;
  size_t buffer_size = sizeof(lv_color_t) * LCD_H_RES * LCD_V_RES;
  buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

  lv_disp_draw_buf_init(&draw_buf, buf, buf1, LCD_H_RES * LCD_V_RES);

  // Initialize display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  // Change the following lines to your display resolution
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Initialize input device driver program
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  delay(100);
  gfx.fillScreen(TFT_BLACK);

  ui_init();

  uart->println("Setup done");
    preferences.begin("wifi-config", false);
  String saved_ssid = preferences.getString("ssid", "");
  String saved_pass = preferences.getString("pass", "");

  WiFi.begin(saved_ssid, saved_pass); // Set WiFi mode to Station
  Serial.printf("Connecting to WiFi SSID: %s, Password: %s\n", saved_ssid.c_str(), saved_pass.c_str());
  preferences.end();
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    lv_label_set_text(ui_wifistr, "Connecting...");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    lv_label_set_text(ui_wifistr, "Offline");
    lv_img_set_src(ui_wifistat, &ui_img_1149142576);
  }
  else
  {
    String wifi = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    lv_label_set_text(ui_wifistr, ip.c_str());
    lv_label_set_text(ui_NetConn, wifi.c_str());
    lv_img_set_src(ui_wifistat, &ui_img_wifi_png);

    Serial.println("Connected to WiFi.");
  }

}

// void print_psram_clk() {
//   uint32_t clk_speed;
//   esp_psram_extram_get_clock_rate(&clk_speed);
//   uart->printf("Actual PSRAM clock frequency: %d MHz\n", clk_speed / 1000000);
// }

void loop()
{
  // Sprawdzaj tylko co 500ms, żeby nie przeciążać I2C
  static unsigned long last_check = 0;
  if (millis() - last_check > 500) {
    last_check = millis();
    if (!i2cScanForAddress(0x18)) {
      uart->println("[TCA9534] Lost connection, reinitializing...");
      ioex.attach(Wire);
      ioex.setDeviceAddress(0x18);
      ioex.config(1, TCA9534::Config::OUT);
      ioex.config(2, TCA9534::Config::OUT);
      ioex_ready = true;
    }
  }

  // print_psram_clk();
  lv_timer_handler(); /* let the GUI do its work */
  delay(1);
  // Check if the screen was touched
 static bool screenDimmed = false;

if (millis() - lastTouchTime > dimTime) {
  if (!screenDimmed) {
    if (ioex_ready) ioex.output(1, TCA9534::Level::L);
    uart->println("Screen dimmed due to inactivity");
    screenDimmed = true;
  }
} else {
  if (screenDimmed) {
    if (ioex_ready) ioex.output(1, TCA9534::Level::H);
    uart->println("Screen brightened due to touch");
    screenDimmed = false;
  }
}

}
