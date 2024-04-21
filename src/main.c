#include "esp_common.h"
#include "freertos/task.h"

// put definition here:

// put function declarations here:
void task_blink(void *ignore);

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map)
    {
    case FLASH_SIZE_4M_MAP_256_256:
        rf_cal_sec = 128 - 5;
        break;

    case FLASH_SIZE_8M_MAP_512_512:
        rf_cal_sec = 256 - 5;
        break;

    case FLASH_SIZE_16M_MAP_512_512:
    case FLASH_SIZE_16M_MAP_1024_1024:
        rf_cal_sec = 512 - 5;
        break;

    case FLASH_SIZE_32M_MAP_512_512:
    case FLASH_SIZE_32M_MAP_1024_1024:
        rf_cal_sec = 1024 - 5;
        break;

    default:
        rf_cal_sec = 0;
        break;
    }

    return rf_cal_sec;
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here.
 * Users can use tasks with priorities from 1 to 9
 * (priority of the freeRTOS timer is 2).
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void user_init(void) // there is a main() function
{
    xTaskCreate(&task_blink, "startup", 2048, NULL, 1, NULL); // Stworzenie zadania wykonywanego przez RTOS
}

// put function definitions here:
void task_blink(void *ignore)
{
    // code here, run once:

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // Ustawienie GPIO2 jako wyjście
    // PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);               // Aktywacja rezystora podciągającego pod GPIO2
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, BIT2); // Umożliwienie ustawienia stanu na wyjściu

    while (true)
    {
        // code here, run repeatedly:
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT2); // Ustawienie stanu niskiego na GPIO2
        vTaskDelay(1000 / portTICK_RATE_MS);

        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, BIT2); // Ustawienie stanu wysokiego na GPIO2
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}