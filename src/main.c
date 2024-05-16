#include "esp_common.h"
#include "freertos/task.h"

// put definition here:
#define UART0 0
#define BAUDRATE 115200                      // Baudrate chciany na lini UART0
#define UART0_DIV (UART_CLK_FREQ / BAUDRATE) // Dzielnik częstotliwości do otrzymania chcianej prędkości transmisji

// put function declarations here:
void print_string(void *ignore);
void task_blink(void *ignore);
void uart0_tx_buffer(char *string, char len);

// put global variables here:
char *test_str = "Your String Here\r\n";

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
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);              // Dezaktywacja restystora podciągającego
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD); // Ustawienie GPIO1 jako linia TX dla UART0

    WRITE_PERI_REG(UART_CLKDIV(0), ((UART0_DIV & UART_CLKDIV_CNT) << UART_CLKDIV_S)); // Ustawienie Baudrate'u
    WRITE_PERI_REG(UART_CONF0(0), ((0x01 & UART_STOP_BIT_NUM) << UART_STOP_BIT_NUM_S) // 1 STOP BIT
                                      | ((0x03 & UART_BIT_NUM) << UART_BIT_NUM_S));   // 8 BIT DATA

    xTaskCreate(&print_string, "test", 2048, NULL, 2, NULL); // Stworzenie zadania wykonywanego przez RTOS

    xTaskCreate(&task_blink, "startup", 2048, NULL, 1, NULL); // Stworzenie zadania wykonywanego przez RTOS
}

// put function definitions here:

/******************************************************************************
 * FunctionName : print_string
 * Description  : broadcast printing string in UART0 every 5s.
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void print_string(void *ignore)
{
    while (true)
    {
        uart0_tx_buffer(test_str, strlen(test_str)); // Wysłanie tekstu przez UART0
        vTaskDelay(5000 / portTICK_RATE_MS);         // Opóźnienie 5s
    }
    vTaskDelete(NULL); // Usunięcie zadania
}

/******************************************************************************
 * FunctionName : task_blink
 * Description  : on board LED blinking every 1s for 1s.
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void task_blink(void *ignore)
{
    // code here, run once:

    // PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);               // Aktywacja rezystora podciągającego pod GPIO2
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // Ustawienie GPIO2 jako wyjście
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, BIT2);      // Umożliwienie ustawienia stanu na wyjściu

    while (true)
    {
        // code here, run repeatedly:
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT2); // Ustawienie stanu niskiego na GPIO2
        vTaskDelay(1000 / portTICK_RATE_MS);         // Opóźnienie 1s

        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, BIT2); // Ustawienie stanu wysokiego na GPIO2
        vTaskDelay(1000 / portTICK_RATE_MS);         // Opóźnienie 1s
    }

    vTaskDelete(NULL); // Usunięcie zadania
}

/******************************************************************************
 * FunctionName : uart_tx_one_char
 * Description  : transmitting one character via UART0.
 * Parameters   : none
 * Returns      : status
 *******************************************************************************/
bool uart_tx_one_char(uint8 uart, uint8 TxChar)
{
    while (true)
    {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S); // Odczytanie ilości byte oczekujących w kolejce TX

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) // Sprawdzenie czy bufor nie jest zapełniony
        {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart), TxChar); // Wysłanie znaku przez UART0
    return OK;
}

/******************************************************************************
 * FunctionName : uart0_write_char
 * Description  : sending character with recognition of escape sequences.
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void uart0_write_char(char c)
{
    switch (c)
    {
    case '\n': // Przejście na początek następnej lini
        uart_tx_one_char(UART0, '\r');
        uart_tx_one_char(UART0, '\n');
        break;

    case '\r': // Powrót na początek linii
        break;

    case '\t': // Przeskok o kilka wierszy
        uart_tx_one_char(UART0, '\t');
        break;

    default: // Wysłanie znaku
        uart_tx_one_char(UART0, c);
        break;
    }
}

/******************************************************************************
 * FunctionName : uart0_tx_buffer
 * Description  : printing string in UART0, char by char;
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void uart0_tx_buffer(char *string, char len)
{
    while (len > 0)
    {
        uart0_write_char(*string++);
        len--;
    }
}