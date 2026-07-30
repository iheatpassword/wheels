#include "uart.h"
#include <stdarg.h>
#include <stdint.h>

static void uart_putchar(UART_Regs* uart, char c)
{
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

static void uart_putuint(UART_Regs* uart, uint32_t num, uint8_t base)
{
    char buf[32];
    int8_t i = 0;

    if (num == 0) {
        uart_putchar(uart, '0');
        return;
    }

    while (num > 0) {
        uint8_t digit = (uint8_t)(num % base);
        if (digit < 10) {
            buf[i++] = (char)('0' + digit);
        } else {
            buf[i++] = (char)('a' + digit - 10);
        }
        num /= base;
    }

    while (i > 0) {
        uart_putchar(uart, buf[--i]);
    }
}

static uint32_t uart_pow10(uint8_t exp)
{
    uint32_t result = 1;
    for (uint8_t i = 0; i < exp; i++) {
        result *= 10;
    }
    return result;
}

static void uart_putfloat(UART_Regs* uart, float num, uint8_t width, uint8_t precision)
{
    char buf[32];
    int8_t i = 0;
    uint8_t is_neg = 0;
    uint32_t int_part;
    uint32_t frac_part;
    uint8_t pad_len;

    if (precision == 0) {
        precision = 6;
    }

    if (num < 0.0f) {
        is_neg = 1;
        num = -num;
    }

    int_part = (uint32_t)num;

    uint32_t pow_val = uart_pow10(precision);
    frac_part = (uint32_t)((num - (float)int_part) * (float)pow_val + 0.5f);

    if (frac_part >= pow_val) {
        int_part++;
        frac_part -= pow_val;
    }

    if (int_part == 0) {
        buf[i++] = '0';
    } else {
        int8_t j = 0;
        char int_buf[16];
        uint32_t temp = int_part;
        while (temp > 0) {
            int_buf[j++] = (char)('0' + (temp % 10));
            temp /= 10;
        }
        while (j > 0) {
            buf[i++] = int_buf[--j];
        }
    }

    buf[i++] = '.';

    if (frac_part == 0) {
        for (uint8_t j = 0; j < precision; j++) {
            buf[i++] = '0';
        }
    } else {
        int8_t j = 0;
        char frac_buf[16];
        uint32_t temp_frac = frac_part;
        while (temp_frac > 0) {
            frac_buf[j++] = (char)('0' + (temp_frac % 10));
            temp_frac /= 10;
        }

        for (int8_t k = precision - j; k > 0; k--) {
            buf[i++] = '0';
        }

        while (j > 0) {
            buf[i++] = frac_buf[--j];
        }
    }

    buf[i] = '\0';

    pad_len = width - i - is_neg;
    if (pad_len > 0) {
        for (uint8_t j = 0; j < pad_len; j++) {
            uart_putchar(uart, ' ');
        }
    }

    if (is_neg) {
        uart_putchar(uart, '-');
    }

    uart_puts(uart, buf);
}

void uart_printf(UART_Regs* uart, char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    while (*fmt != '\0') {
        if (*fmt == '%') {
            fmt++;
            uint8_t width = 0;
            uint8_t precision = 0;

            while (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
                fmt++;
            }

            if (*fmt == '.') {
                fmt++;
                while (*fmt >= '0' && *fmt <= '9') {
                    precision = precision * 10 + (*fmt - '0');
                    fmt++;
                }
            }

            switch (*fmt) {
                case 'd':
                case 'i':
                    uart_putint(uart, va_arg(args, int), 10);
                    break;
                case 'u':
                    uart_putuint(uart, va_arg(args, uint32_t), 10);
                    break;
                case 'x':
                case 'X':
                    uart_putuint(uart, va_arg(args, uint32_t), 16);
                    break;
                case 'c':
                    uart_putchar(uart, (char)va_arg(args, int));
                    break;
                case 's':
                    uart_puts(uart, va_arg(args, char*));
                    break;
                case 'f':
                    uart_putfloat(uart, (float)va_arg(args, double), width, precision);
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

/* JustFloat 协议：发送 float 数组 + 帧尾 {0x00, 0x00, 0x80, 0x7f}
 * 格式：[float0 小端4字节] [float1] ... [floatN] [00 00 80 7F]
 * 用 union 保证 float 内存布局正确，避免指针别名问题 */
void uart_justfloat(UART_Regs* uart, float *data, uint8_t count)
{
    union { float f; uint8_t b[4]; } u;
    for (uint8_t i = 0; i < count; i++) {
        u.f = data[i];
        uart_putchar(uart, (char)u.b[0]);
        uart_putchar(uart, (char)u.b[1]);
        uart_putchar(uart, (char)u.b[2]);
        uart_putchar(uart, (char)u.b[3]);
    }
    /* 帧尾：IEEE 754 +INF (小端: 00 00 80 7f) */
    uart_putchar(uart, (char)0x00);
    uart_putchar(uart, (char)0x00);
    uart_putchar(uart, (char)0x80);
    uart_putchar(uart, (char)0x7f);
}
