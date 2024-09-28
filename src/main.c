#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_common.h"
#include "uart.h"
#include "hw_timer.h"

// put definition here:
#define UART0 0
#define BAUDRATE 115200                      // Baudrate chciany na lini UART0
#define UART0_DIV (UART_CLK_FREQ / BAUDRATE) // Dzielnik częstotliwości do otrzymania chcianej prędkości transmisji

#define UART_EVEN_PARITY 0 // bit nieparzystości
#define UART_ODD_PARITY 1  // bit parzystości

#define UART_STOP_1BIT 1   // 1 bit stopu
#define UART_STOP_1_2BIT 2 // 1.5 bit stopu
#define UART_STOP_2BIT 3   // 2 bit stopu

#define UART_BIT_NUM_5 0 // 5 bit danych
#define UART_BIT_NUM_6 1 // 6 bit danych
#define UART_BIT_NUM_7 2 // 7 bit danych
#define UART_BIT_NUM_8 3 // 8 bit danych

// put Task declarations here:
void print_data_from_RX(void *ignore);
void reciveing_uart(void *ignore);
void print_string(void *ignore);
void task_blink(void *ignore);

// put function declarations here:
void uart0_tx_buffer(char *string, char len);
bool uart_tx_one_char(uint8 uart, uint8 TxChar);

// put global variables here:
char *test_str = "Your String Here\r\n";
char *callback_str = "You send: ";
uint8_t fifo_len = 0;
uint8_t buf_idx = 0;
uint8_t Rcv_data[256];
///////////////////////////////////////////////////////////////////////////////////////
///*                                    WSKAZÓWKA:                                  *//
// Dla zmiennych wykorzystywanych w przerwaniach stosować "volatile", przy braku     //
// zastosowania "volatile", a gdy zmienna nie jest użyta w głównej pętli programu    //
// kompilator może zoptymalizować program o zmienną (wyciąć ją).                     //
///////////////////////////////////////////////////////////////////////////////////////
volatile uint8_t Rcv_data_len = 0;
volatile uint8_t timer_interrupt_count = 0;

void timer_intr_handler(void) // funkcja obsługująca przerwanie od timera
{
    timer_interrupt_count++;

    if (timer_interrupt_count >= 6) // Jeśli upłynie 60 sekund
    {
        // Wykonaj działanie po 60 sekundach
        timer_interrupt_count = 0;
        uart0_tx_buffer("Mineła 1 min\r\n", strlen("Mineła 1 min\r\n"));
    }

    // hw_timer_arm(50, 0);
}

void uart_intr_handler(void *arg) // funkcja obsługująca przerwanie od UART0
{
    // uint32_t uart_intr_status = READ_PERI_REG(UART_INT_ST(UART0));

    // if (uart_intr_status & UART_RXFIFO_FULL_INT_ST) // Sprawdzenie, czy przerwanie zostało wywołane przez pełny bufor RX FIFO
    // {
    // os_printf("RX FIFO Full Interrupt!\n"); // for debug com. -> obsługa pełnego bufora RX

    uint32 fifo_cnt = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; // Odczytanie ilości byte oczekujących w kolejce RX

    while (fifo_cnt)
    // while (READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
    {
        // uint8_t data = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF; // Odczytaj pojedynczy bajt z FIFO UART i przechowaj go w buforze data
        // uart_tx_one_char(UART0, data);                         // Prześlij odebrane dane z powrotem na UART (echo)

        Rcv_data[Rcv_data_len] = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF; // Odczytaj pojedynczy bajt z FIFO UART i przechowaj go w buforze Rcv_data
        Rcv_data_len++;                                                  // Zwiększenie długości bufora (zajętości)

        fifo_cnt = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; // Odczytaj ponownie w celu zaktualizowania wartości zmiennej
    }

    WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR); // Wyczyść flagę przerwania pełnego bufora RX FIFO
    // }

    /*
    // DO THE SAME AS FUN reciveing_uart BUT WITHOUT FUN WHICH ARE DEDICATED TO TASK OF FREERTOS
    uint8_t *data = (uint8_t *)malloc(1024);                                                      // dynamiczna alokacji pamięci dla zmiennej
    uint32 fifo_cnt = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; // Odczytanie ilości bajtów oczekujących w kolejce RX

    if (fifo_cnt) // Sprawdź, czy są dane do odczytu
    {
        for (uint8_t i = 0; i < fifo_cnt; i++)
        {
            data[i] = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF; // Odczytaj pojedynczy bajt z FIFO UART i przechowaj go w buforze data
        }

        data[fifo_cnt] = '\0'; // Zakończ łańcuch danych null-terminatorem (opcjonalne)

        // Wysłanie danych przez UART
        uart0_tx_buffer(callback_str, strlen(callback_str));
        uart0_tx_buffer((char *)data, fifo_cnt);
        uart0_tx_buffer("\r\n", strlen("\r\n"));

        // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);    // for debug
        // os_printf("Task3 stack: %d\n\r", dat);
    }

    WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR); // Wyczyszczenie flag przerwań RX FIFO
    free(data);                                                                               // Zwalnianie pamięci
    */
}

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
    // GPIO INIT (chose GPIO function for UART Pin):
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);              // Dezaktywacja restystora podciągającego
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD); // Ustawienie GPIO1 jako linia TX dla UART0
    PIN_PULLUP_EN(PERIPHS_IO_MUX_U0RXD_U);               // Aktywacja restystora podciągającego
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD); // Ustawienie GPIO3 jako linia RX dla UART0
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS); // Aktywacja hardware flow control

    // UART INIT:
    UART_SetBaudrate(UART0, BAUDRATE); // Ustawienie Baudrate'u
    // WRITE_PERI_REG(UART_CLKDIV(0), ((UART0_DIV & UART_CLKDIV_CNT) << UART_CLKDIV_S)); // Ustawienie Baudrate'u
    // UART_SetParity(UART0, UART_ODD_PARITY); // sprawdzania poprawności transmisji po przez kontrolę parzystości (EVEN || ODD)

    UART_SetStopBits(UART0, UART_STOP_1_2BIT); // 1 STOP BIT
    UART_SetWordLength(UART0, UART_BIT_NUM_8); // 8 BIT DATA
    // WRITE_PERI_REG(UART_CONF0(0), ((0x01 & UART_STOP_BIT_NUM) << UART_STOP_BIT_NUM_S) // 1 STOP BIT
    //   | ((0x03 & UART_BIT_NUM) << UART_BIT_NUM_S));   // 8 BIT DATA

    // UART_SetLineInverse(UART0, UART_RXD_INV);   // odwrócenie sygnału transmisji na lini RX
    // UART_SetPrintPort(UART0); // wybór UART'a wykorzystywanego do funkcji os_printf
    // SET_PERI_REG_MASK(UART_CONF0(UART0), UART_LOOPBACK); // aktywowanie funkcji Loopback (dane z TX są natychmiast przekazywane z powrotem do RX w ramach tego samego urządzenia)
    // CLEAR_PERI_REG_MASK(UART_CONF0(UART0), UART_LOOPBACK);// wyłączenie funkcji Loopback
    // UART_SetFlowCtrl(UART0, UART_HardwareFlowCtrl_RTS | UART_HardwareFlowCtrl_CTS, UART_RTS_LOW_LEVEL_THRESH); // ustawienie sprzętowego Flow Control na UART0

    // Interrupt INIT:
    UART_SetIntrEna(UART0, UART_RXFIFO_FULL_INT_ENA);    // Ustawienie przerwań dla pełnego bufora RX FIFO
    UART_intr_handler_register(uart_intr_handler, NULL); // Rejestracja obsługi przerwań UART
    ETS_UART_INTR_ENABLE();                              // Włączenie przerwań UART

    // TIMER INIT:
    hw_timer_init();
    hw_timer_set_func(timer_intr_handler); // Rejestracja obsługi przerwań
    hw_timer_arm(10000000, 1);             // Przerwanie co 10 sekundę (Timer FRC1 23bit max 26.84 sekundy)

    xTaskCreate(&print_data_from_RX, "Task4", 4096, NULL, 3, NULL); // Stworzenie zadania "print_data_from_RX (TX)" wykonywanego przez RTOS

    // xTaskCreate(&reciveing_uart, "Task3", 4096, NULL, 3, NULL); // Stworzenie zadania "reciveing_uart (RX/TX)" wykonywanego przez RTOS

    xTaskCreate(&print_string, "Task2", 2048, NULL, 2, NULL); // Stworzenie zadania "print_string (TX)" wykonywanego przez RTOS

    xTaskCreate(&task_blink, "Task1", 2048, NULL, 1, NULL); // Stworzenie zadania "task_blink" wykonywanego przez RTOS
}

// put function definitions here:
/******************************************************************************
 * FunctionName : print_data_from_RX
 * Description  : print data reciving from line RX every 1s.
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void print_data_from_RX(void *ignore)
{
    while (true)
    {
        // Wysłanie tekstu przez UART0
        if (Rcv_data_len)
        {
            // os_printf("Data len: %d\n\r", Rcv_data_len); //for debug

            uart0_tx_buffer(callback_str, strlen(callback_str));
            for (uint8_t i = 0; i < Rcv_data_len; i++)
            {
                uart_tx_one_char(UART0, Rcv_data[i]);
                Rcv_data[i] = '\0'; // Czyszczenie bufora za pomocą null-terminatora
            }
            uart0_tx_buffer("\r\n", strlen("\r\n"));

            Rcv_data_len = 0; // Wyzerowanie jeżeli wszystkie dane odebrano
        }

        vTaskDelay(1000 / portTICK_RATE_MS); // Opóźnienie 5s

        // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);   // for debug
        // os_printf("Task2 stack: %d\n\r", dat);
    }
    vTaskDelete(NULL); // Usunięcie zadania
}
/******************************************************************************
 * FunctionName : reciveing_uart
 * Description  : echo data reciving from line RX.
 * Parameters   : none
 * Returns      : none
 *******************************************************************************/
void reciveing_uart(void *ignore)
{
    uint8_t *data = (uint8_t *)malloc(1024); // dynamiczna alokacji pamięci dla zmiennej
    while (true)
    {
        fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; // Odczytanie ilości bajtów oczekujących w kolejce RX

        if (fifo_len > 0) // Sprawdź, czy są dane do odczytu
        {
            for (uint8_t i = 0; i < fifo_len; i++)
            {
                data[i] = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF; // Odczytaj pojedynczy bajt z FIFO UART i przechowaj go w buforze data
            }

            WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR); // Wyczyszczenie flag przerwań RX FIFO

            data[fifo_len] = '\0'; // Zakończ łańcuch danych null-terminatorem (opcjonalne)

            // Wysłanie danych przez UART
            uart0_tx_buffer(callback_str, strlen(callback_str));
            uart0_tx_buffer((char *)data, fifo_len);
            uart0_tx_buffer("\r\n", strlen("\r\n"));

            // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);    // for debug
            // os_printf("Task3 stack: %d\n\r", dat);
        }

        // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);    // for debug
        // os_printf("Task3 stack: %d\n\r", dat);
        vTaskDelay(100 / portTICK_RATE_MS); // Opóźnienie 0.1s
    }

    free(data);        // Zwalnianie pamięci
    vTaskDelete(NULL); // Usunięcie taska
}
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

        // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);   // for debug
        // os_printf("Task2 stack: %d\n\r", dat);
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

        // uint16_t dat = uxTaskGetStackHighWaterMark(NULL);    // for debug
        // os_printf("Task1 stack: %d\n\r", dat);
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