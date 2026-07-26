#include "uart.h"
#include <stdarg.h>
#include <stdint.h>


static void uart_putchar(UART_Regs* uart, char c)
{
    /* 等待发送保持寄存器为空（非 FIFO 模式下同样适用） */
    while (DL_UART_Main_isTXFIFOFull(uart)) {
        ;
    }
    DL_UART_Main_transmitData(uart, (uint8_t)c);
}

static void uart_puts(UART_Regs* uart, const char* str)
{
    while (*str != '\0') {
        uart_putchar(uart, *str++);
    }
}

void uart_send_string(UART_Regs* uart, char* str)
{
    uart_puts(uart, str);
}

static void uart_putint(UART_Regs* uart, int num, uint8_t base)
{
    char buf[32];
    int8_t i = 0;
    uint8_t is_neg = 0;
    uint32_t unsigned_num;

    if (num < 0 && base == 10) {
        is_neg = 1;
        unsigned_num = (uint32_t)(-num);
    } else {
        unsigned_num = (uint32_t)num;
    }

    if (unsigned_num == 0) {
        uart_putchar(uart, '0');
        return;
    }

    while (unsigned_num > 0) {
        uint8_t digit = (uint8_t)(unsigned_num % base);
        if (digit < 10) {
            buf[i++] = (char)('0' + digit);
        } else {
            buf[i++] = (char)('a' + digit - 10);
        }
        unsigned_num /= base;
    }

    if (is_neg) {
        uart_putchar(uart, '-');
    }

    while (i > 0) {
        uart_putchar(uart, buf[--i]);
    }
}

void uart_printf(UART_Regs* uart, char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    while (*fmt != '\0') {
        if (*fmt == '%') {
            fmt++;
            switch (*fmt) {
                case 'd':
                case 'i':
                    uart_putint(uart, va_arg(args, int), 10);
                    break;
                case 'u':
                    uart_putint(uart, (int)va_arg(args, uint32_t), 10);
                    break;
                case 'x':
                case 'X':
                    uart_putint(uart, va_arg(args, int), 16);
                    break;
                case 'c':
                    uart_putchar(uart, (char)va_arg(args, int));
                    break;
                case 's':
                    uart_puts(uart, va_arg(args, char*));
                    break;
                case '%':
                    uart_putchar(uart, '%');
                    break;
                default:
                    uart_putchar(uart, '%');
                    uart_putchar(uart, *fmt);
                    break;
            }
        } else {
            uart_putchar(uart, *fmt);
        }
        fmt++;
    }

    va_end(args);
}
