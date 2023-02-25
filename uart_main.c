#include <gpio.h>
#include <stm32.h>
#include <string.h>
#include <irq.h>
#include "buffer.h"

// **************** USART ******************************

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | USART_CR1_PS)

#define USART_StopBits_1 0x0000

#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS USART_CR3_RTSE
#define USART_FlowControl_CTS USART_CR3_CTSE

#define USART_New_Input (USART2->SR & USART_SR_RXNE)

#define USART_Free_Output (USART2->SR & USART_SR_TXE)

// **************** Buttons ***************************

// active on low
#define JOY_GPIO GPIOB
#define JOY_LEFT_PIN 3
#define JOY_RIGHT_PIN 4
#define JOY_UP_PIN 5
#define JOY_DOWN_PIN 6
#define JOY_PUSH_PIN 10

#define JOY_BITMASK 0x478

// active on low
#define USR_GPIO GPIOC
#define USR_PIN 13

#define USR_BITMASK 0x2000

// active on high
#define AT_MODE_GPIO GPIOA
#define AT_MODE_PIN 0

#define AT_MODE_BITMASK 0x1

// ***************** Misc ******************************

#define HSI_HZ 16000000U

#define PCLK1_HZ HSI_HZ

#define BAUD_RATE 9600U

#define IN_MSG_LEN 3

#define OUT_MSG_MAX_LEN 17

// ****************** LEDs *******************************

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA
#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5

#define RedLEDon() \
    RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() \
    RED_LED_GPIO->BSRR = 1 << RED_LED_PIN
#define RedLEDflip() \
    RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16 * (1 & (RED_LED_GPIO->ODR >> RED_LED_PIN)))

#define GreenLEDon() \
    GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff() \
    GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN
#define GreenLEDflip() \
    GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16 * (1 & (GREEN_LED_GPIO->ODR >> GREEN_LED_PIN)))

#define BlueLEDon() \
    BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff() \
    BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN
#define BlueLEDflip() \
    BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16 * (1 & (BLUE_LED_GPIO->ODR >> BLUE_LED_PIN)))

#define Green2LEDon() \
    GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
    GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)
#define Green2LEDflip() \
    GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16 * (1 & (GREEN2_LED_GPIO->ODR >> GREEN2_LED_PIN)))

// **************  last handled interrupt to avoid starvation *********

uint32_t prev_but_state;
int in_buff[3];
int in_size = 0;
buffer out_buff;

char dma_out[OUT_MSG_MAX_LEN] = {};

char dma_in;

// ***************** Code *********************************

void setNewMessage(int button_code, int new_state_code, char *msg)
{
    switch (button_code)
    {
    case AT_MODE_PIN:
        strncpy(msg, "MODE ", 15);
        break;
    case JOY_LEFT_PIN:
        strncpy(msg, "LEFT ", 15);
        break;
    case JOY_RIGHT_PIN:
        strncpy(msg, "RIGHT ", 15);
        break;
    case JOY_DOWN_PIN:
        strncpy(msg, "DOWN ", 15);
        break;
    case JOY_UP_PIN:
        strncpy(msg, "UP ", 15);
        break;
    case JOY_PUSH_PIN:
        strncpy(msg, "FIRE ", 15);
        break;
    case USR_PIN:
        strncpy(msg, "USER ", 15);
        break;
    default:
        strncpy(msg, "ERR ", 15);
    }
    while (*msg != '\0')
    {
        msg++;
    }
    switch (new_state_code)
    {
    case 0:
        strcpy(msg, "PRESSED\r\n");
        break;
    case 1:
        strcpy(msg, "RELEASED\r\n");
        break;
    default:
        strcpy(msg, "ERR\r\n");
    }
}

void handleOutput(uint32_t but_changed, uint32_t new_state)
{
    int shift = 0;
    uint32_t prev_but = prev_but_state;
    uint32_t but_change = but_changed;
    while (but_change)
    {
        if (but_change & 1)
        {
            uint32_t dma_priority = NVIC_EncodePriority(3, 14, 0);
            irq_level_t prot_level = IRQprotect(dma_priority);
            //   we just drop old messages from the buffer instead of checking if it's full
            //   todo is this ok?
            buff_push(&out_buff, shift);
            // push 0 if pressed, 1 if released
            buff_push(&out_buff, prev_but & 1);
            IRQunprotect(prot_level);
            break;
        }
        shift++;
        but_change >>= 1;
        prev_but >>= 1;
    }
    prev_but_state = new_state | (prev_but_state & ~but_changed);
}

void ledOn(int *buff)
{
    switch (buff[1])
    {
    case 'R':
        RedLEDon();
        break;
    case 'G':
        GreenLEDon();
        break;
    case 'B':
        BlueLEDon();
        break;
    case 'g':
        Green2LEDon();
        break;
    default:
        return;
    }
}

void ledOff(int *buff)
{
    switch (buff[1])
    {
    case 'R':
        RedLEDoff();
        break;
    case 'G':
        GreenLEDoff();
        break;
    case 'B':
        BlueLEDoff();
        break;
    case 'g':
        Green2LEDoff();
        break;
    default:
        return;
    }
}

void ledFlip(int *buff)
{
    switch (buff[1])
    {
    case 'R':
        RedLEDflip();
        break;
    case 'G':
        GreenLEDflip();
        break;
    case 'B':
        BlueLEDflip();
        break;
    case 'g':
        Green2LEDflip();
        break;
    default:
        return;
    }
}

void handleInput(int *buff)
{
    if (buff[0] != 'L')
        return;
    switch (buff[2])
    {
    case '1':
        ledOn(buff);
        break;
    case '0':
        ledOff(buff);
        break;
    case 'T':
        ledFlip(buff);
        break;
    default:
        return;
    }
    // todo set given led, change above defines to dinamically choose led maybe, wathc out for green2 being on a different part of board
}

// input pin handlers

void EXTI0_IRQHandler(void)
{
    uint32_t new_state = (AT_MODE_GPIO->IDR & (1U << AT_MODE_PIN));
    handleOutput((1U << AT_MODE_PIN), new_state);
    EXTI->PR = EXTI_PR_PR0;
}

void EXTI3_IRQHandler(void)
{
    uint32_t new_state = (~JOY_GPIO->IDR & (1U << JOY_LEFT_PIN));
    handleOutput((1U << JOY_LEFT_PIN), new_state);
    EXTI->PR = EXTI_PR_PR3;
}

void EXTI4_IRQHandler(void)
{
    uint32_t new_state = (~JOY_GPIO->IDR & (1U << JOY_RIGHT_PIN));
    handleOutput((1U << JOY_RIGHT_PIN), new_state);
    EXTI->PR = EXTI_PR_PR4;
}

// todo maybe nop to let the edge fall or rise completely
void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR5)
    {
        uint32_t new_state = (~JOY_GPIO->IDR & (1U << JOY_UP_PIN));
        handleOutput((1U << JOY_UP_PIN), new_state);
        EXTI->PR = EXTI_PR_PR5;
    }
    else
    {
        uint32_t new_state = (~JOY_GPIO->IDR & (1U << JOY_DOWN_PIN));
        handleOutput((1U << JOY_DOWN_PIN), new_state);
        EXTI->PR = EXTI_PR_PR6;
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10)
    {
        uint32_t new_state = (~JOY_GPIO->IDR & (1U << JOY_PUSH_PIN));
        handleOutput((1U << JOY_PUSH_PIN), new_state);
        EXTI->PR = EXTI_PR_PR10;
    }
    else
    {
        uint32_t new_state = (~USR_GPIO->IDR & (1U << USR_PIN));
        handleOutput((1U << USR_PIN), new_state);
        EXTI->PR = EXTI_PR_PR13;
    }
}

// after send
void DMA1_Stream6_IRQHandler()
{
    /* Odczytaj zgłoszone przerwania DMA1. */
    uint32_t isr = DMA1->HISR;
    if (isr & DMA_HISR_TCIF6)
    {
        memset(dma_out, '\0', OUT_MSG_MAX_LEN);
        /* Obsłuż zakończenie transferu
        w strumieniu 6. */
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        /* Jeśli jest coś do wysłania,
        wystartuj kolejną transmisję. */
        if (!buff_empty(&out_buff))
        {
            int button_code = buff_pop(&out_buff);
            int state_code = buff_pop(&out_buff);
            setNewMessage(button_code, state_code, dma_out);
            DMA1_Stream6->M0AR = (uint32_t)dma_out;
            DMA1_Stream6->NDTR = strlen(dma_out);
            DMA1_Stream6->CR |= DMA_SxCR_EN;
        }
    }
}

// after recieve
void DMA1_Stream5_IRQHandler()
{
    /* Odczytaj zgłoszone przerwania DMA1. */
    uint32_t isr = DMA1->HISR;
    if (isr & DMA_HISR_TCIF5)
    {
        /* Obsłuż zakończenie transferu
        w strumieniu 5. */
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
        in_size = dma_in == 'L' ? 1 : in_size + 1;
        in_buff[in_size - 1] = dma_in;
        if (in_size == IN_MSG_LEN)
        {
            handleInput(in_buff);
            in_size = 0;
        }
        /* Ponownie uaktywnij odbieranie. */
        DMA1_Stream5->M0AR = (uint32_t)&dma_in;
        DMA1_Stream5->NDTR = 1;
        DMA1_Stream5->CR |= DMA_SxCR_EN;
    }
}

// bitmap of pins where a 1 means the button is engaged regardless of wether
// the buttons is active on high or low
uint32_t buttonState()
{
    uint32_t ans = 0;
    ans |= (~JOY_GPIO->IDR & JOY_BITMASK);
    ans |= (~USR_GPIO->IDR & USR_BITMASK);
    ans |= (AT_MODE_GPIO->IDR & AT_MODE_BITMASK);
    return ans;
}

int main()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // for ext. interrupts
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    __NOP();
    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();
    GPIOoutConfigure(RED_LED_GPIO,
                     RED_LED_PIN,
                     GPIO_OType_PP,
                     GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    GPIOoutConfigure(GREEN_LED_GPIO,
                     GREEN_LED_PIN,
                     GPIO_OType_PP,
                     GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    GPIOoutConfigure(BLUE_LED_GPIO,
                     BLUE_LED_PIN,
                     GPIO_OType_PP,
                     GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    GPIOoutConfigure(GREEN2_LED_GPIO,
                     GREEN2_LED_PIN,
                     GPIO_OType_PP,
                     GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    // in pins config for interrupts

    // Joystick, engaged on low
    GPIOinConfigure(JOY_GPIO, JOY_UP_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    GPIOinConfigure(JOY_GPIO, JOY_DOWN_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    GPIOinConfigure(JOY_GPIO, JOY_LEFT_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    GPIOinConfigure(JOY_GPIO, JOY_RIGHT_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    GPIOinConfigure(JOY_GPIO, JOY_PUSH_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    // USER, engaged low
    GPIOinConfigure(USR_GPIO, USR_PIN,
                    GPIO_PuPd_UP, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    // AT_MODE, engaged HIGH
    GPIOinConfigure(AT_MODE_GPIO, AT_MODE_PIN,
                    GPIO_PuPd_DOWN, EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);

    // uart pins config
    GPIOafConfigure(GPIOA,
                    2,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_USART2);

    // pull up to avoid random transfer at boot
    GPIOafConfigure(GPIOA,
                    3,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_UP,
                    GPIO_AF_USART2);

    prev_but_state = buttonState();
    buff_init(&out_buff);

    USART2->CR1 = USART_Mode_Rx_Tx | USART_WordLength_8b | USART_Parity_No;

    USART2->CR2 = USART_StopBits_1;

    USART2->CR3 = USART_FlowControl_None;

    USART2->BRR = (PCLK1_HZ + (BAUD_RATE / 2U)) / BAUD_RATE;

    // enable dma

    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

    // config DMA tx

    // 4U becvasue channel 4, why << 25? becaues of chsel register bits 25-27
    DMA1_Stream6->CR = 4U << 25 |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_DIR_0 |
                       DMA_SxCR_TCIE;

    // data register to write to
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

    // config dma rx

    DMA1_Stream5->CR = 4U << 25 |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_TCIE;

    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6 |
                  DMA_HIFCR_CTCIF5;

    NVIC_SetPriorityGrouping(3);

    uint32_t dma_priority = NVIC_EncodePriority(3, 14, 0);

    NVIC_SetPriority(DMA1_Stream6_IRQn, dma_priority);
    NVIC_SetPriority(DMA1_Stream5_IRQn, dma_priority);

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    // starts listening here
    USART2->CR1 |= USART_Enable;

    // enable interrupts

    EXTI->PR = EXTI_PR_PR0 |
               EXTI_PR_PR3 |
               EXTI_PR_PR4 |
               EXTI_PR_PR5 |
               EXTI_PR_PR6 |
               EXTI_PR_PR10 |
               EXTI_PR_PR13;

    // no subpriority
    NVIC_SetPriorityGrouping(3);

    uint32_t but_priority = NVIC_EncodePriority(3, 15, 0);

    NVIC_SetPriority(EXTI0_IRQn, but_priority);
    NVIC_SetPriority(EXTI3_IRQn, but_priority);
    NVIC_SetPriority(EXTI4_IRQn, but_priority);
    NVIC_SetPriority(EXTI9_5_IRQn, but_priority);
    NVIC_SetPriority(EXTI15_10_IRQn, but_priority);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // init recieve
    DMA1_Stream5->M0AR = (uint32_t)&dma_in;
    DMA1_Stream5->NDTR = 1;
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    while (1)
    {

        // todo disable interrupts for recieveing button inputs
        irq_level_t prot_level = IRQprotect(dma_priority);
        if (!buff_empty(&out_buff) &&
            (DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
            (DMA1->HISR & DMA_HISR_TCIF6) == 0)
        {
            int button_code = buff_pop(&out_buff);
            int state_code = buff_pop(&out_buff);
            IRQunprotect(prot_level);
            setNewMessage(button_code, state_code, dma_out);
            DMA1_Stream6->M0AR = (uint32_t)dma_out;
            DMA1_Stream6->NDTR = strlen(dma_out);
            DMA1_Stream6->CR |= DMA_SxCR_EN;
        }
        else
        {
            IRQunprotect(prot_level);
        }
        __WFI();
    }

    return 1;
}