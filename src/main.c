#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

void internal_clock();

/*
//ENABLE NECESSARY PORTS
void enable_ports(void) {
    //enable gpiob and gpioc
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2)) | (3 << (4 * 2) | (3 << (5 * 2)))); // reset pins 0-5
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)) | (3 << (9 * 2)) | (3 << (10 * 2))); // reset pins 6-10

    GPIOB->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (3 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2))); // set pins 0-5 to output
    GPIOB->MODER |= ((1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (8 * 2)) | (1 << (9 * 2)) | (1 << (10 * 2))); // set pins 6-10 to output

    GPIOC->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2))); // reset pins 4-7
    GPIOC->MODER |= ((1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2))); // set pins 4-7 to output
    GPIOC->OTYPER |= (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7); // set pins 4-7 to open drain

    GPIOC->MODER &= ~((3 << 0) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2))); // reset pins 0-3 to make them input
    GPIOC->PUPDR |= ((1 << 0) | (1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (3 * 2))); // set pins 0-3 as pulled up
}
    */

//SETTING UP AMPLITUDE CONTROL WITH ADC + POTENTIOMETER
uint32_t amplitude = 2048;

void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= (3 << (1 * 2));
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));

    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    ADC1->CHSELR = ADC_CHSELR_CHSEL1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

//BOXCAR AVERAGING FOR ADC
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void TIM2_IRQHandler(){
    TIM2->SR &= ~TIM_SR_UIF; //acknowledge interrupt
    ADC1->CR |= ADC_CR_ADSTART; //start adc
    while(!(ADC1->ISR & ADC_ISR_EOC)); //wait till end of eoc conversion

    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
        amplitude = bcsum / BCSIZE;
}

void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->ARR = 1000 - 1; //reload = 1000
    TIM2->PSC = 4800 - 1; //psc = 4800
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}
//END AMPLITUDE CONTROL



//SETTING UP SIN WAVE AND DAC
#define M_PI 3.14159265358979323846
#define N 1000
#define RATE 20000
short int wavetable[N];
int step = (1<<16); //step = 65536 for 20hz wave
int offset = 0;

void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= (3 << (4 * 2));
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    DAC->CR &= ~(DAC_CR_TSEL1); //tim6 is all 0's, tsel1 is all 1's
    DAC->CR |= DAC_CR_TEN1; //enable trigger

    DAC->CR |= DAC_CR_EN1; //enable dac
}

void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF; //acknowledge interrupt

    offset += step;
    if (offset >= (N << 16)){
        offset -= (N << 16);
    }

    int samp = wavetable[offset>>16];
    samp *= amplitude;
    samp >>= 17; 
    samp += 2048;
    DAC->DHR12R1 = samp;
}

void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    //interrupt is invoked 20000 times per second (20kHz)
    TIM6->PSC = 48 - 1; //psc = 48
    TIM6->ARR = (1000000 / RATE) - 1; //arr = 1/rate * 10^6 (because clock src is 1MHz)

    TIM6->CR2 |= TIM_CR2_MMS_1; //update master mode select 

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}
//END SIN WAVE AND DAC

int main(void){
    internal_clock();
    //enable_ports();
    setup_adc();
    init_tim2();
    init_wavetable();
    setup_dac();
    init_tim6();

    while(1);
}