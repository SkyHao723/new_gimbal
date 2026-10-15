/**
 * ============================================================================
 * 极简串口（直接操作寄存器，不依赖 Arduino 的 Serial 类）
 *
 * 为什么不用 Arduino 的 Serial/Serial1：
 *   实测在本 core 版本(4.3.0)下，Serial1.println() 完全无输出，
 *   而直接写 USART2 寄存器正常。为可靠起见改用寄存器直驱。
 *
 * 也顺便支持从串口收命令（用于 E/D/T/V 等控制命令）。
 * ============================================================================
 */
#ifndef MYUART_H
#define MYUART_H

#include <Arduino.h>

/* USART2: TX=PA2, RX=PA3 */
class MyUart {
public:
    void begin(uint32_t baud = 115200)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

        /* PA2 复用推挽 50MHz */
        GPIOA->CRL &= ~(0xF << 8);
        GPIOA->CRL |=  (0xB << 8);
        /* PA3 浮空输入 */
        GPIOA->CRL &= ~(0xF << 12);
        GPIOA->CRL |=  (0x4 << 12);

        /* 72MHz -> PCLK1=36MHz；BRR = 36e6/baud */
        USART2->BRR = (uint16_t)((36000000UL + baud / 2) / baud);
        USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    }

    /* ---- 输出 ---- */
    void putc(char c)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)c;
    }

    void puts(const char *s) { while (*s) putc(*s++); }

    void puti(int32_t v)
    {
        char buf[12]; int i = 0;
        if (v < 0) { putc('-'); v = -v; }
        if (v == 0) { putc('0'); return; }
        while (v > 0 && i < 11) { buf[i++] = (char)('0' + v % 10); v /= 10; }
        while (i--) putc(buf[i]);
    }

    /* 定点输出，1 位小数 */
    void putf1(float v)
    {
        if (v < 0) { putc('-'); v = -v; }
        int32_t ip = (int32_t)v;
        int32_t fp = (int32_t)((v - (float)ip) * 10.0f + 0.5f);
        if (fp > 9) fp = 9;
        puti(ip); putc('.'); puti(fp);
    }

    /* 定点输出，3 位小数 */
    void putf3(float v)
    {
        if (v < 0) { putc('-'); v = -v; }
        int32_t ip = (int32_t)v;
        int32_t fp = (int32_t)((v - (float)ip) * 1000.0f + 0.5f);
        if (fp > 999) fp = 999;
        puti(ip); putc('.');
        if (fp < 100) putc('0');
        if (fp < 10)  putc('0');
        puti(fp);
    }

    void nl() { putc('\r'); putc('\n'); }

    /* ---- 接收 ---- */
    bool available()
    {
        return (USART2->SR & USART_SR_RXNE) ? true : false;
    }
    char getc()
    {
        while (!(USART2->SR & USART_SR_RXNE));
        return (char)(USART2->DR & 0xFF);
    }

private:
    /* 空 */
};

extern MyUart uart;

#endif  /* MYUART_H */
