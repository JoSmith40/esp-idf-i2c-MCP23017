/*
Dieser Code wurde mit Hilfe von natürlicher Intelligenz erstellt. ;-)
Eine KI wurde zum Prüfen und Verbessern des Codes benutzt.

Erstellt: 04.2025
Autor: JoSmith40

Getestet mit LilyGo T-Display-S3 (ESP32-S3R8 = 16MB Flash, 320x170)
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mcp23017.h"

#define I2C_SCL 17
#define I2C_SDA 18
#define MCP23017_ADDR 0x20
#define I2C_FREQ_HZ 100000 // 400 kHz (100 ... 800 kHz)

static const char *TAG = "main.c says: ";

void app_main(void)
{
      esp_err_t ret;
      // Deklaration des i2c-Bus-Handles
      i2c_master_bus_handle_t bus_handle;

      // I2C-Bus-Konfiguration
      i2c_master_bus_config_t bus_config = {
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .i2c_port = -1,
          .intr_priority = 0,
          .scl_io_num = I2C_SCL,
          .sda_io_num = I2C_SDA,
          .glitch_ignore_cnt = 7,
          .flags = {
              .enable_internal_pullup = 1,
          }};

      // Erstellen des I2C-Bus
      ret = i2c_new_master_bus(&bus_config, &bus_handle);
      if (ret != ESP_OK)
      {
            ESP_LOGE(TAG, "I2C-Bus-Initialisierung fehlgeschlagen");
            return;
      }
      ESP_LOGV(TAG, "I2C-Bus initialisiert");

      // Declare the MCP23017 to the initialization
      mcp23017_t mcp;
      mcp.i2c_addr = MCP23017_ADDR;
      mcp.bus_handle = bus_handle;
      mcp.i2c_freq = I2C_FREQ_HZ;
      mcp.int_pin = 5; // Beispiel-Interrupt-Pin (anpassen)
      mcp.use_interrupts = true;

      // Initializing the MCP23017
      mcp23017_err_t err = mcp23017_init(&mcp);
      if (err != MCP23017_ERR_OK)
      {
            ESP_LOGE(TAG, "MCP23017 Initialisierung fehlgeschlagen: %s", mcp23017_err_to_string(err));
            return;
      }
      ESP_LOGI(TAG, "MCP23017 initialisiert: %s", mcp23017_err_to_string(err));
      ESP_LOGI(TAG, "MCP23017 Address: 0x%02X", mcp.i2c_addr);
      ESP_LOGI(TAG, "I2C Speed: %u kHz", (unsigned int)(mcp.i2c_freq / 1000));

      // Configuration MCP23017 for System
      mcp23017_write_register(&mcp, MCP23017_IODIR, GPIOA, 0xFF); // Setze alle GPIOA-Pins als Eingänge
      mcp23017_write_register(&mcp, MCP23017_IODIR, GPIOB, 0x00); // Setze alle GPIOB-Pins als Ausgänge
      mcp23017_write_register(&mcp, MCP23017_IPOL, GPIOA, 0x00);  // Setze IPOL-Register, Port A
      mcp23017_write_register(&mcp, MCP23017_IPOL, GPIOB, 0x00);  // Setze IPOL-Register, Port B
      mcp23017_write_register(&mcp, MCP23017_GPPU, GPIOA, 0x00);  // Setze GPPU-Register, Port A
      mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOA, 0x00);  // Setze GPIOA-Pins auf LOW
      mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOB, 0xFF);  // Setze GPIOB-Pins auf HIGH
      mcp23017_write_register(&mcp, MCP23017_GPINTEN, GPIOA, 0xFF); // Aktiviere Interrupts für alle GPIOA-Pins
      mcp23017_write_register(&mcp, MCP23017_GPINTEN, GPIOB, 0x00); // Deaktiviere Interrupts für alle GPIOB-Pins
      mcp23017_write_register(&mcp, MCP23017_INTCON, GPIOA, 0xFF); // Setze INTCON-Register, Port A
      mcp23017_write_register(&mcp, MCP23017_DEFVAL, GPIOA, 0xFF); // Setze DEFVAL-Register, Port A
      mcp23017_write_register(&mcp, MCP23017_IOCON, GPIOA, 0x20); // Setze IOCON-Register
      uint8_t value;
      mcp23017_read_register(&mcp, MCP23017_INTCAP, GPIOA, &value); // Lese INTCAP-Register clear Interrupts
      ESP_LOGI(TAG, "INTCAP-Wert beim Interrupt: 0x%02X", value);
      if (err != MCP23017_ERR_OK)
      {
            ESP_LOGE(TAG, "MCP23017 Konfiguration fehlgeschlagen: %s", mcp23017_err_to_string(err));
            return;
      }
      ESP_LOGI(TAG, "MCP23017 konfiguriert %s", mcp23017_err_to_string(err));
      ESP_LOGI(TAG, "MCP23017 Port A = INPUTS, Interrupts akktiv LOW");
      ESP_LOGI(TAG, "MCP23017 Port B = OUTPUTS");
      // Hier können Sie mit dem MCP23017 arbeiten

      // Endlose Schleife, um das Programm am Laufen zu halten
      while (1)
      {
            vTaskDelay(pdMS_TO_TICKS(1000));
      }
}
