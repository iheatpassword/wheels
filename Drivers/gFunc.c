#include "gFunc.h"
volatile uint32_t gMillis=0;

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            gMillis++;//although TIMER_0 is count down, gMillis auto increment
            break;
        default:
            break;    
    }
}

//return systick ms
extern inline uint32_t millis(void)
{
    return gMillis;
}

//uart echo
void UART_0_INST_IRQHandler(void)
{
    uint8_t echoData=0;
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            DL_GPIO_togglePins(LED_PORT, LED_led0_PIN);
            echoData=DL_UART_Main_receiveData(UART_0_INST);
            DL_UART_Main_transmitData(UART_0_INST, echoData);
            break;
        default:
            break;    
    }
}

