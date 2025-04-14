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
#include "esp_timer.h"

#define I2C_SCL 17
#define I2C_SDA 18
#define MCP23017_ADDR 0x20
#define I2C_FREQ_HZ 400000 // 400 kHz (100 ... 800 kHz)

// Zeitmessungsvariablen
int64_t start_time, end_time, elapsed_time;
int64_t total_time = 0;
int iterations = 0;
const int print_interval = 5000; // Ausgabe alle 5000 Iterationen

#define NUM_INPUTS 8
uint16_t counter[NUM_INPUTS] = {0}; // Array für die Zähler
float kWh[NUM_INPUTS] = {0.0};      // Array für kWh Werte

uint8_t captureValue = 0;

static const char *TAG = "main.c says";

// Globale Variablen für I2C und MCP23017
static i2c_master_bus_config_t bus_config;
static i2c_master_bus_handle_t bus_handle;
static mcp23017_t mcp;

void processImpulses(uint8_t captureValue)
{
      static uint8_t lastCapture = 0;
      for (int i = 0; i < NUM_INPUTS; i++)
      {
            // Bit war vorher 1 und ist jetzt 0 => fallende Flanke
            if (!(captureValue & (1 << i)) && (lastCapture & (1 << i)))
            {
                  counter[i]++;                 // Zähler erhöhen
                  kWh[i] = counter[i] / 1000.0; // optional: 1000 Impulse = 1 kWh
            }
      }
      lastCapture = captureValue;
}

/**
 * Konfiguriert den I2C-Bus
 */
static void config_i2c_bus(void)
{
      bus_config = (i2c_master_bus_config_t) {
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .i2c_port = -1,
          .intr_priority = 0,
          .scl_io_num = I2C_SCL,
          .sda_io_num = I2C_SDA,
          .glitch_ignore_cnt = 7,
          .flags = {
              .enable_internal_pullup = 1,
          }};
}

/**
 * Initialisiert den I2C-Bus
 */
static esp_err_t init_i2c(void)
{
      esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
      if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C-Bus-Initialisierung fehlgeschlagen");
      } else {
            ESP_LOGV(TAG, "I2C-Bus initialisiert");
      }
      return err;
}

/**
 * Konfiguriert das MCP23017-Gerät
 */
static void config_mcp23017(void)
{
      mcp.i2c_addr = MCP23017_ADDR;
      mcp.bus_handle = bus_handle;
      mcp.i2c_freq = I2C_FREQ_HZ;
      mcp.int_pin = 5; // Adjust as required
      mcp.use_interrupts = true;
}

/**
 * Initialisiert das MCP23017-Gerät
 */
static mcp23017_err_t init_mcp23017(void)
{
      mcp23017_err_t mcp_err = mcp23017_init(&mcp);
      if (mcp_err != MCP23017_ERR_OK) {
            ESP_LOGE(TAG, "MCP23017 Initialization failed: %s", mcp23017_err_to_string(mcp_err));
      } else {
            ESP_LOGI(TAG, "MCP23017 initialized: %s", mcp23017_err_to_string(mcp_err));
            ESP_LOGI(TAG, "MCP23017 Address: 0x%02X", mcp.i2c_addr);
            ESP_LOGI(TAG, "I2C Speed: %u kHz", (unsigned int)(mcp.i2c_freq / 1000));
      }
      return mcp_err;
}

/**
 * Konfiguriert die Ports und Register des MCP23017
 */
static mcp23017_err_t configure_mcp23017_ports(void)
{
      mcp23017_err_t mcp_err = MCP23017_ERR_OK;

      // Configuration MCP23017 for System, 8x Inputs, 8x Outputs
      mcp23017_write_register(&mcp, MCP23017_IODIR, GPIOA, 0xFF);   // Alle Pins von Port A als Eingänge
      mcp23017_write_register(&mcp, MCP23017_IODIR, GPIOB, 0x00);   // Alle Pins von Port B als Ausgänge
      mcp23017_write_register(&mcp, MCP23017_IPOL, GPIOA, 0x00);    // Polarität Port A unverändert
      mcp23017_write_register(&mcp, MCP23017_IPOL, GPIOB, 0x00);    // Polarität Port B unverändert
      mcp23017_write_register(&mcp, MCP23017_GPPU, GPIOA, 0xFF);    // Pull-ups auf Kanal A aktivieren
      mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOA, 0x00);    // Setze GPIOA-Pins auf LOW
      mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOB, 0xFF);    // Setze GPIOB-Pins auf HIGH
      mcp23017_write_register(&mcp, MCP23017_GPINTEN, GPIOA, 0xFF); // Aktiviere Interrupts für alle GPIOA-Pins
      mcp23017_write_register(&mcp, MCP23017_GPINTEN, GPIOB, 0x00); // Deaktiviere Interrupts für alle GPIOB-Pins
      mcp23017_write_register(&mcp, MCP23017_INTCON, GPIOA, 0xFF);  // Interrupt auf Änderung
      mcp23017_write_register(&mcp, MCP23017_DEFVAL, GPIOA, 0xFF);  // Vergleichswert für Interrupts (nötig für INTCON)
      mcp23017_write_register(&mcp, MCP23017_IOCON, GPIOA, 0x20);   // Interrupt auf aktives Low setzen, Mirror deaktiv

      uint8_t value;
      mcp23017_read_register(&mcp, MCP23017_INTCAP, GPIOA, &value); // clear Interrupts
      ESP_LOGI(TAG, "INTCAP-Wert beim Interrupt: 0x%02X", value);   // Optional: Log the INTCAP value

      return mcp_err;
}

void app_main(void)
{
      // I2C-Bus konfigurieren und initialisieren
      config_i2c_bus();
      if (init_i2c() != ESP_OK) {
            return;
      }

      // MCP23017 konfigurieren und initialisieren
      config_mcp23017();
      if (init_mcp23017() != MCP23017_ERR_OK) {
            return;
      }

      // MCP23017 Ports configurieren
      mcp23017_err_t mcp_err = configure_mcp23017_ports();
      if (mcp_err != MCP23017_ERR_OK) {
            ESP_LOGE(TAG, "MCP23017 Konfiguration fehlgeschlagen: %s", mcp23017_err_to_string(mcp_err));
            return;
      }
      ESP_LOGI(TAG, "MCP23017 config ok %s", mcp23017_err_to_string(mcp_err));
      ESP_LOGI(TAG, "MCP23017 Port A = INPUTS, Interrupts active LOW");
      ESP_LOGI(TAG, "MCP23017 Port B = OUTPUTS");
      ESP_LOGI(TAG, "");

      // Endless loop to keep the program running
      while (1) {
            start_time = esp_timer_get_time(); // Startzeit in Mikrosekunden

            mcp23017_read_register(&mcp, MCP23017_GPIO, GPIOA, &captureValue); // Read port A
            processImpulses(captureValue);                                     // Process impulses
            mcp23017_write_register(&mcp, MCP23017_GPIO, GPIOB, captureValue); // Set GPIOB pins port B

            // Binary representation of GPIO values (optional)
            char bin_str[9];
            for (int i = 0; i < 8; i++) {
                bin_str[i] = (captureValue & (0x80 >> i)) ? '1' : '0';
            }
            bin_str[8] = '\0';

            // Timing in µs
            end_time = esp_timer_get_time(); // End time in microseconds
            elapsed_time = end_time - start_time; // Calculation of elapsed time
            total_time += elapsed_time; // Update total time
            iterations++; // Increase iterations

            // Output statistics
            if (iterations % print_interval == 0) {
                  ESP_LOGI(TAG, "Port B: 0x%02X (Bits: %s)", captureValue, bin_str);
                  ESP_LOGI(TAG, "Time measurement: Current iteration: %lld µs, Average: %lld µs",
                           elapsed_time, total_time / iterations);
            }
      }
}
