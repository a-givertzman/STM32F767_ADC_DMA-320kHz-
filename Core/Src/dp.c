#include <stm32f7xx_hal.h>
#include <stdio.h>
#include <stdint.h>
#define ADC_COUNT 2

struct DpResult{
    int err;
    int setpoint;
};

void initSPI(){
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Включаем тактовый сигнал для SPI1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Включаем тактовый сигнал для GPIOA

    //Сбрасываем конфигурационные биты в нули
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
   
    //SCK
    GPIOA->MODER |= 0x02<<GPIO_MODER_MODER5_Pos; //Максимальная частота
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; //Установка режима pull-push
  
    //MISO
    GPIOA->MODER |= 0x01<<GPIO_MODER_MODER6_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;
  
    //MOSI
    GPIOA->MODER |= 0x02<<GPIO_MODER_MODER7_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;


    //Настройка SPI
    SPI1->CR1 = (0<<11)     // Размер кадра 8 бит
    | (0 << SPI_CR1_LSBFIRST_Pos)     // MSB first
    | (1 << SPI_CR1_SSM_Pos)          // Программное управление SS
    | (1 << SPI_CR1_SSI_Pos)          // SS в высоком состоянии
    | (0x03 << SPI_CR1_BR_Pos)        // Скорость передачи: F_PCLK/64
    | (1 << SPI_CR1_MSTR_Pos);        // Режим "мастер"
    SPI1->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI
}

int receiveData(){
    //SPI1->DR = 0;
    while (!(SPI1->SR & SPI_SR_RXNE)); // Ждем, пока буфер приемника не заполнится
    return SPI1->DR; // Возвращаем прочитанные данные
}

int sendData(data){
    while(!(SPI1->SR & SPI_SR_TXE)); //Ждём, пока буфер передатчика не освободится
    SPI1->DR = data; // Записываем данные для передачи в регистр данных SPI1
}

int communicateWithDp(int command){
    sendData(command); //отправка команды по SPI
    return receiveData(); // получение ответа от потенциометра
}

struct DpResult dpReadSetpoint(){
    struct DpResult dpResult;
    int command = 0b10000000; //контрольный байт, зависит от потенциометра
    int err = communicateWithDp(command);
    int setpoint = 0;
    if (err > 0){
        dpResult.err = err;
    }
    else{
        dpResult.setpoint = setpoint;
    }

    return dpResult;
}

struct DpResult dpWriteSetpoint(int newSetpoint){
    struct DpResult dpResult;
    int command = 0b10000000 | (newSetpoint & 0xFF);
    int err = communicateWithDp(command);

    if (err > 0){
        dpResult.err = err;
    }
    else{
        dpResult.setpoint = newSetpoint;
    }

    return dpResult;
}

int main(void){

    initSPI();

    struct DpResult adcSetpoint[ADC_COUNT];

    for(int i = 0; i < ADC_COUNT; i++){
        adcSetpoint[i] = dpReadSetpoint();
    }

    int newSetpoint = 10;
    struct DpResult whiteResult = dpWriteSetpoint(newSetpoint);

    if (whiteResult.err > 0) {
        printf("Произошла ошибка при установке новой точки.\n");
    } else {
        printf("Новая установленная точка: %d\n", newSetpoint);
    }
}