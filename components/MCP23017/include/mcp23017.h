#ifndef MCP23017_H
#define MCP23017_H

#include <stdio.h>
#include <stdint.h>  // Für uint8_t
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

// Port A register
#define MCP23017_IODIRA		0x00
#define MCP23017_IPOLA 		0x02
#define MCP23017_GPINTENA 	0x04
#define MCP23017_DEFVALA 	0x06
#define MCP23017_INTCONA 	0x08
#define MCP23017_IOCONA 	0x0A
#define MCP23017_GPPUA 		0x0C
#define MCP23017_INTFA 		0x0E
#define MCP23017_INTCAPA 	0x10
#define MCP23017_GPIOA 		0x12
#define MCP23017_OLATA 		0x14

// Port B register
#define MCP23017_IODIRB 	0x01
#define MCP23017_IPOLB 		0x03
#define MCP23017_GPINTENB 	0x05
#define MCP23017_DEFVALB 	0x07
#define MCP23017_INTCONB 	0x09
#define MCP23017_IOCONB 	0x0B
#define MCP23017_GPPUB 		0x0D
#define MCP23017_INTFB 		0x0F
#define MCP23017_INTCAPB 	0x11
#define MCP23017_GPIOB 		0x13
#define MCP23017_OLATB 		0x15

#define MCP23017_DEFAULT_ADDR	0x20

/**
 * @brief  Specifies an error code returned by functions in the MCP23017 AP
 * @note  The error codes are defined as follows:
 *        - MCP23017_ERR_OK:     No error
 *        - MCP23017_ERR_CONFIG: Configuration error
 *        - MCP23017_ERR_INSTALL: Installation error
 *        - MCP23017_ERR_FAIL:   General failure
 * @return An error code
 */
typedef enum {
   MCP23017_ERR_OK      = 0x00,
   MCP23017_ERR_CONFIG  = 0x01,
   MCP23017_ERR_INSTALL = 0x02,
   MCP23017_ERR_FAIL    = 0x03
} mcp23017_err_t;

/**
 * @brief Konvertiert einen MCP23017-Fehlercode in einen lesbaren String
 * @param err der Fehlercode
 * @return ein lesbarer String, der den Fehler beschreibt
 */
const char *mcp23017_err_to_string(mcp23017_err_t err);

/**
*   @brief  Specifies register index
*   @note   a generic register which
*   can point to either group A or
*   group B depending on an offset that
*   can be applied.
* @return  A register index
*/
typedef enum {
    MCP23017_IODIR	= 0x00,
    MCP23017_IPOL	   = 0x01,
    MCP23017_GPINTEN	= 0x02,
    MCP23017_DEFVAL	= 0x03,
    MCP23017_INTCON	= 0x04,
    MCP23017_IOCON	= 0x05,
    MCP23017_GPPU	   = 0x06,
    MCP23017_INTF	   = 0x07,
    MCP23017_INTCAP	= 0x08,
    MCP23017_GPIO	   = 0x09,
    MCP23017_OLAT	   = 0x0A
} mcp23017_reg_t;

/**
 * @brief Specifies a GPIO pin group
 * @note The MCP23017 has two groups A or B
 */
typedef enum {
    GPIOA = 0x00,
    GPIOB = 0x01
} mcp23017_gpio_t;

/**
 * @brief  Specifies a interface configuration
 * @note interface configuration
   for the MCP23017. This structure is
   used to initialize the I2C bus and
   the MCP23017 device.
 */
typedef struct {
   uint8_t i2c_addr;                   // I2C-Adresse des MCP23017
   uint32_t i2c_freq;                  // I2C-Frequenz (Hz)
   bool enable_internal_pullups;       // Interne Pull-ups aktivieren
   uint8_t int_pin;                    // Interrupt-Pin
   bool use_interrupts;                // Interrupts verwenden
   i2c_master_bus_handle_t bus_handle; // I2C-Bus-Handle
   i2c_master_dev_handle_t dev_handle; // I2C-Geräte-Handle
} mcp23017_t;

/**
 * @brief  Function prototypes
 */
mcp23017_err_t mcp23017_init(mcp23017_t *mcp);
mcp23017_err_t mcp23017_write_register(mcp23017_t *mcp, mcp23017_reg_t reg, mcp23017_gpio_t group, uint8_t v);
mcp23017_err_t mcp23017_read_register(mcp23017_t *mcp, mcp23017_reg_t reg, mcp23017_gpio_t group, uint8_t *data);
mcp23017_err_t mcp23017_set_bit(mcp23017_t *mcp, uint8_t bit, mcp23017_reg_t reg, mcp23017_gpio_t group);
mcp23017_err_t mcp23017_clear_bit(mcp23017_t *mcp, uint8_t bit, mcp23017_reg_t reg, mcp23017_gpio_t group);

#endif /* MCP23017_H */
