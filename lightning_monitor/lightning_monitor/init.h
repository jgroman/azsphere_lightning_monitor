
#ifndef INIT_H
#define INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*   Included Headers
*******************************************************************************/

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

#include <applibs/i2c.h>

#include "main.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define I2C_ISU             PROJECT_ISU2_I2C
#define I2C_BUS_SPEED       I2C_BUS_SPEED_STANDARD
#define I2C_TIMEOUT_MS      (100u)

#define I2C_ADDR_OLED       (0x3C)

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

/** @brief Initialize signal handlers.
*
* Set up SIGTERM termination handler.
*
* @return 0 on success, -1 otherwise.
*/
int
init_handlers(void);

/** @brief Initialize peripherals.
*
* Initialize all peripherals used by this project.
*
* @return 0 on success, -1 otherwise.
*/
int
init_peripherals(void);

/**
 *
 */
void
init_shutdown(void);


#ifdef __cplusplus
}
#endif

#endif  // INIT_H

/* [] END OF FILE */
