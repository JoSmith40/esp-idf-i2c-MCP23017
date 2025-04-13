#include <stdio.h>
#include "mcp23017.h"

// #include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_log.h"

static const char *TAG = "mcp23017.c says";

/**
 * @brief Konvertiert einen MCP23017-Fehlercode in einen lesbaren String
 * @param err der Fehlercode
 * @return ein lesbarer String, der den Fehler beschreibt
 */
const char *mcp23017_err_to_string(mcp23017_err_t err)
{
    switch (err)
    {
    case MCP23017_ERR_OK:
        return "NO ERROR";
    case MCP23017_ERR_INSTALL:
        return "INSTALLATION ERROR";
    case MCP23017_ERR_FAIL:
        return "GENERAL ERROR";
    default:
        return "UNKNOWN ERROR";
    }
}

/**
 * @brief Converts generic register and group (A/B) to register address
 * @param reg the generic register index
 * @param group the group (A/B) to compute offset
 * @return The register address specified by the parameters
 */
uint8_t mcp23017_register(mcp23017_reg_t reg, mcp23017_gpio_t group)
{
    return (group == GPIOA) ? (reg << 1) : (reg << 1) | 1;
}

/**
 * @brief Initializes the MCP23017
 * @param mcp the MCP23017 interface structure
 * @return an error code or MCP23017_ERR_OK
 */
mcp23017_err_t mcp23017_init(mcp23017_t *mcp)
{
    esp_err_t ret;

    // Device Configuration
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mcp->i2c_addr,
        .scl_speed_hz = mcp->i2c_freq,
    };

    // Create the device handle
    ret = i2c_master_bus_add_device(mcp->bus_handle, &dev_config, &mcp->dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERR: I2C device initialization failed: %s (ESP-Err: %s)",
                 mcp23017_err_to_string(MCP23017_ERR_INSTALL), esp_err_to_name(ret));
        i2c_del_master_bus(mcp->bus_handle);
        return MCP23017_ERR_INSTALL;
    }
    ESP_LOGV(TAG, "I2C device initialized: %s", mcp23017_err_to_string(MCP23017_ERR_OK));

    // only use for interrupts
    if (mcp->use_interrupts && mcp->int_pin >= 0)
    {
        // Hier können Sie den Interrupt-Code hinzufügen
        // z.B. GPIO konfigurieren, Interrupt-Register des MCP23017 konfigurieren
        ESP_LOGV(TAG, "Interrupts werden verwendet, INT-Pin: %d", mcp->int_pin);

        // Beispiel für die Interrupt-Konfiguration (muss an Ihre Anforderungen angepasst werden)
        // gpio_config_t io_conf = {
        //     .pin_bit_mask = (1ULL << mcp->int_pin),
        //     .mode = GPIO_MODE_INPUT,
        //     .pull_up_en = GPIO_PULLUP_DISABLE,
        //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
        //     .intr_type = GPIO_INTR_NEGEDGE,
        // };
        // gpio_config(&io_conf);
    }

    return MCP23017_ERR_OK;
}

/**
 * @brief a value to an MCP23017 register
 * @param mcp the MCP23017 interface structure
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @param v the value to write to the register
 * @return an error code or MCP23017_ERR_OK if no error encountered
 */
mcp23017_err_t mcp23017_write_register(mcp23017_t *mcp, mcp23017_reg_t reg, mcp23017_gpio_t group, uint8_t v)
{
    uint8_t r = mcp23017_register(reg, group);
    uint8_t write_buf[2] = {r, v};

    esp_err_t ret = i2c_master_transmit(mcp->dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR: unable to write to register: %s (ESP-Err: %s)",
                 mcp23017_err_to_string(MCP23017_ERR_FAIL), esp_err_to_name(ret));
        return MCP23017_ERR_FAIL;
    }
    return MCP23017_ERR_OK;
}

/**
 * @brief a value to an MCP23017 register
 * @param mcp the MCP23017 interface structure
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @param data a pointer to an 8 bit value to be read from the device
 * @return an error code or MCP23017_ERR_OK if no error encountered
 */
/*
// Alternate lesen! ###################
mcp23017_err_t mcp23017_read_register(mcp23017_t *mcp, mcp23017_reg_t reg, mcp23017_gpio_t group, uint8_t *data) {
    uint8_t r = mcp23017_register(reg, group);
    esp_err_t ret;

    // ####### Registeradresse schreiben und Daten in einem Schritt lesen #######
    ret = i2c_master_transmit_receive(mcp->dev_handle, &r, 1, data, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: unable to read reg %02x from address %02x", r, mcp->i2c_addr);
        return MCP23017_ERR_FAIL;
    }
    return MCP23017_ERR_OK;
}
*/
mcp23017_err_t mcp23017_read_register(mcp23017_t *mcp, mcp23017_reg_t reg, mcp23017_gpio_t group, uint8_t *data)
{
    uint8_t r = mcp23017_register(reg, group);
    esp_err_t ret;
    // Registeradresse schreiben
    ret = i2c_master_transmit(mcp->dev_handle, &r, 1, -1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR: unable to write address %02x to read reg %02x", mcp->i2c_addr, r);
        return MCP23017_ERR_FAIL;
    }
    // Daten aus dem Register lesen
    ret = i2c_master_receive(mcp->dev_handle, data, 1, -1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR: unable to read reg %02x from address %02x", r, mcp->i2c_addr);
        return MCP23017_ERR_FAIL;
    }
    return MCP23017_ERR_OK;
}

/**
 * @brief a bit in a current register value
 * @param mcp address of the MCP23017 data structure
 * @param bit The number of the bit to set
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @return an error code or MCP23017_ERR_OK if no error encountered
 */
mcp23017_err_t mcp23017_set_bit(mcp23017_t *mcp, uint8_t bit, mcp23017_reg_t reg, mcp23017_gpio_t group)
{
    uint8_t current_value;
    if (mcp23017_read_register(mcp, reg, group, &current_value) != MCP23017_ERR_OK)
    {
        uint8_t r = mcp23017_register(reg, group);
        ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x", r);
        return MCP23017_ERR_FAIL;
    }
    current_value |= 1 << bit;
    if (mcp23017_write_register(mcp, reg, group, current_value) != MCP23017_ERR_OK)
    {
        uint8_t r = mcp23017_register(reg, group);
        ESP_LOGE(TAG, "ERROR: unable to write new value %02X to register %02x", current_value, r);
        return MCP23017_ERR_FAIL;
    }
    return MCP23017_ERR_OK;
}

/**
 * @brief Clears a bit from a current register value
 * @param mcp address of the MCP23017 data structure
 * @param bit The number of the bit to set
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @return an error code or MCP23017_ERR_OK if no error encountered
 */
mcp23017_err_t mcp23017_clear_bit(mcp23017_t *mcp, uint8_t bit, mcp23017_reg_t reg, mcp23017_gpio_t group)
{
    uint8_t current_value;
    if (mcp23017_read_register(mcp, reg, group, &current_value) != MCP23017_ERR_OK)
    {
        uint8_t r = mcp23017_register(reg, group);
        ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x", r);
        return MCP23017_ERR_FAIL;
    }
    current_value &= ~(1 << bit);
    if (mcp23017_write_register(mcp, reg, group, current_value) != MCP23017_ERR_OK)
    {
        uint8_t r = mcp23017_register(reg, group);
        ESP_LOGE(TAG, "ERROR: unable to write new value %02X to register %02x", current_value, r);
        return MCP23017_ERR_FAIL;
    }
    return MCP23017_ERR_OK;
}

/**
 * @brief Delete the MCP23017
 * @param mcp the MCP23017 interface structure
 * @return an error code or MCP23017_ERR_OK if no error encountered
 */
mcp23017_err_t mcp23017_deinit(mcp23017_t *mcp)
{
    esp_err_t ret;

    // Geräte-Handle freigeben
    ret = i2c_master_bus_rm_device(mcp->dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C-Geräte-Deinitialisierung fehlgeschlagen");
        return MCP23017_ERR_FAIL;
    }

    // Bus-Handle freigeben
    ret = i2c_del_master_bus(mcp->bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C-Bus-Deinitialisierung fehlgeschlagen");
        return MCP23017_ERR_FAIL;
    }

    return MCP23017_ERR_OK;
}
