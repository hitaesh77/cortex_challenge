#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

void internal_clock();

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

//ENABLE USART FOR PRINTF
void init_usart5() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;

    //PC12
    GPIOC->MODER &= ~(3 << (12 * 2));
    GPIOC->MODER |= (2 << (12 * 2)); 
    GPIOC->AFR[1] |= (2 << (4 * (12 - 8))); //AF2 with PC12

    //PD2
    GPIOD->MODER &= ~(3 << (2 * 2));
    GPIOD->MODER |= (2 << (2 * 2));
    GPIOD->AFR[0] |= (2 << (4 * 2)); //AF2 with PD2

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5->CR1 &= ~USART_CR1_UE; //disable USART
    USART5->CR1 &= ~USART_CR1_M; //8 bit data size
    USART5->CR2 &= ~USART_CR2_STOP; //1 stop bit
    USART5->CR1 &= ~USART_CR1_PCE; //no parity
    USART5->CR1 &= ~USART_CR1_OVER8; //oversampling by 16
    USART5->BRR = 48000000 / 115200; //baud rate 115200
    USART5->CR1 |= USART_CR1_TE; //enable transmitter
    USART5->CR1 |= USART_CR1_RE; //enable receiver
    USART5->CR1 |= USART_CR1_UE; //enable USART
}

int __io_putchar(int c) {
    while (!(USART5->ISR & USART_ISR_TXE));  // Wait until the TX buffer is empty
    USART5->TDR = (c & 0xFF);  // Send character to USART5
    return c;
}
//END USART FOR PRINTF


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

    if (bcn >= BCSIZE) {
        bcn = 0;
        amplitude = bcsum / BCSIZE;
    }
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
int sample = 0;

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

    sample = wavetable[offset>>16];
    sample *= amplitude;
    sample >>= 17; 
    sample += 2048;
    DAC->DHR12R1 = sample;
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



//DATA AQUISTION AND PROCESSING
#define BUFFER_SIZE 5  //moving average of 5 samples
int buffer[BUFFER_SIZE]; //circular buffer
int buffer_index = 0; //buffer index
int moving_avg = 0; //moving average

int calc_mov_avg(){
    int sum = 0;
    for(int i = 0; i < BUFFER_SIZE; i++){
        sum += buffer[i];
    }
    return sum / BUFFER_SIZE;
}

// Timer 3 interrupt handler
void TIM3_IRQHandler(void) {
    TIM3->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
    buffer[buffer_index] = abs(sample - 2048);
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;

    moving_avg = calc_mov_avg();
    //printf("moving average: %d\n", moving_avg);
}

void init_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    //interrupt is invoked every 10ms (100Hz)
    TIM3->PSC = 48 - 1;  //psc = 48
    TIM3->ARR = 10000 - 1;  //arr = 10000

    // Enable update interrupt for Timer 2
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;    
}
//END DATA AQUISTION AND PROCESSING



//ALERT SYSTEM
#define THRESHOLD 400 //threshold for seizure detection

void init_leds(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (10 * 2))); // reset pins 8 and 10
    GPIOB->MODER |= ((1 << (8 * 2)) | (1 << (10 * 2))); // set pins 8 and 10 to output
}

//function for toggling LEDs
void togglexn(GPIO_TypeDef *port, int n) {
    port->ODR ^= (1 << n);
  }

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;  // Clear interrupt flag

    if (moving_avg > THRESHOLD) {
        printf("Seizure detected!\n");
        spi1_display1("Seizure Detected!");
        if(GPIOB->ODR & (1 << 10)) {
            togglexn(GPIOB, 8);
            togglexn(GPIOB, 10);
        } else {
            togglexn(GPIOB, 8);
        }
    } else {
        printf("Normal\n");
        spi1_display1("Normal           ");
        if(GPIOB->ODR & (1 << 8)) {
            togglexn(GPIOB, 8);
            togglexn(GPIOB, 10);
        } else {
            togglexn(GPIOB, 10);
        }
    }
}

void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    //interrupt is invoked every 1s (1Hz)
    TIM7->PSC = 48000 - 1;  //psc = 48000
    TIM7->ARR = 1000 - 1;  //arr = 1000

    // Enable update interrupt for Timer 7
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;    
}
//END ALERT SYSTEM



//SPI SYSTEM FOR FUN
void init_spi1() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //set clock

    GPIOA->MODER &= ~((3 << (5 * 2)) | (3 << (7 * 2)) | (3 << (15 * 2))); // reset pins
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (7 * 2)) | (2 << (15 * 2)); //set output

    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (7 * 4))); //set channels for pins 5,7 (AF0)
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4)); //set channels for pins 15 (AF0)
    
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_TXDMAEN |  
                (SPI_CR2_DS_0 | SPI_CR2_DS_3);
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_cmd(unsigned int data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
}

void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}

void spi1_init_oled() {
    nano_wait(1000000); 
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0C);
}

void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while (*string != '\0') {
        spi_data(*string);
        string++;
    }
}
//END SPI SYSTEM



int main(void){
    internal_clock();

    //initialize "brain wave" system
    setup_adc();
    init_tim2();
    init_wavetable();
    setup_dac();
    init_tim6();

    //initialize data aquisition and processing
    init_tim3();
    init_usart5();

    //initialize alert system
    init_leds();
    init_tim7();

    //initialize spi system
    init_spi1();
    spi1_init_oled();

    while(1);
}