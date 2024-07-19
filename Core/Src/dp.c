#include <stm32f7xx_hal.h>
#define ADC_COUNT 2

struct DpResult{
    int err;
    int setpoint;
};

struct DigitalPotentiometer{
    int csPin;
};

void initSPI(){

}

void sendCommand(){

}
int receiveData(){

}

void closeSPI(){

}

//SPI connection
int communicateWithDp(int csPin){
    initSPI();
    sendCommand();
    int data = receiveData();
    closeSPI();
    return data;
}

struct DpResult dpReadSetpoint(struct DigitalPotentiometer *dp){
    struct DpResult dpResult;
    int err = communicateWithDp(dp->csPin);
    int setpoint = 0;
    if (err > 0){
        dpResult.err = err;
    }
    else{
        dpResult.setpoint = setpoint;
    }

    return dpResult;
}

struct DpResult dpWriteSetpoint(struct DigitalPotentiometer *dp, int newSetpoint){
    struct DpResult dpResult;
    int err = communicateWithDp(dp->csPin);

    if (err > 0){
        dpResult.err = err;
    }
    else{
        dpResult.setpoint = newSetpoint;
    }

    return dpResult;

}


int main(void){

    struct  DigitalPotentiometer dp;
    dp.csPin = 9; //for example

    struct DpResult adcSetpoint[ADC_COUNT];

    for(int i = 0; i<ADC_COUNT; i++){
        adcSetpoint[i] = dpReadSetpoint(&dp);
    }

    int newSetpoint = 10;
    struct DpResult whiteResult = dpWriteSetpoint(&dp, newSetpoint);

    if (whiteResult.err > 0) {
        printf("Произошла ошибка при установке новой точки.\n");
    } else {
        printf("Новая установленная точка: %d\n", newSetpoint);
    }
}