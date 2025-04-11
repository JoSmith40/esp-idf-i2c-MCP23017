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

void app_main(void)
{
      // Declare the MCP23017 structure to transfer to the initialization
      mcp23017_t mcp;
      mcp.i2c_addr = MCP23017_ADDR;
      mcp.port = 0; // I2C port number
      mcp.sda_pin = I2C_SDA;
      mcp.scl_pin = I2C_SCL;
      mcp.i2c_freq = 400000; // 100 ... 800 kHz
      mcp.enable_internal_pullups = true; // Aktivieren Sie die internen Pull-ups
      mcp.int_pin = 5;       // INT-Pin auf GPIO 5
      mcp.use_interrupts = true;

      // Initialisieren Sie den MCP23017
      // Die I2C-Bus-Initialisierung geschieht intern in mcp23017_init
      mcp23017_err_t err = mcp23017_init(&mcp);
      if (err != MCP23017_ERR_OK)
      {
            ESP_LOGE("MCP23017", "Initialisierung fehlgeschlagen");
            return;
      }

      ESP_LOGI("MCP23017", "Initialisierung erfolgreich");

      // Hier können Sie mit dem MCP23017 arbeiten
      // Beispiel: Alle Ausgänge von GPIOA auf HIGH setzen
      mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOA, 0xFF);

      // Endlose Schleife, um das Programm am Laufen zu halten
      while (1)
      {
            vTaskDelay(pdMS_TO_TICKS(1000));
      }
}
