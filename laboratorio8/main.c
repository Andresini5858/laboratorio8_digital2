
//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"

#define XTAL 16000000 //reloj externo de 16MHz

char bandera = 0; //bandera para iniciar el semáforo
uint32_t i; //variable para delay
int valor; //Valor para leer estado del push button

void setup(void); //Función del setup
void setupTimer0(void); //Función del timer0
void setupUART(void); //Función para UART0
void delay1ms(void); //Función para delay de 1ms
void delay(uint32_t msec); //Función para delay en ms

int main(void){
    setup(); //Llamar al setup
    setupTimer0(); //Llamar al setup para el Timer0
    setupUART(); //Llamar al setup para UART
    //Loop forever.
    while(1){
        valor = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4); //Leer estado del Push button
        if (valor == 0){ //Si está presionado limpiar la bandera del semáforo
            if (bandera == 1){
                bandera = 0; //Limpiar bandera
            }
        }
        if (valor == 0 && bandera == 0){ //Si se presionó el botón y la bandera es 0
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //Encender led verde
            delay(3000); //delay de 3s
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //Encender led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //Encender led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //Encender led verde
            delay(200); //delay de 200ms
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
            delay(1000); //delay de 1s

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, GPIO_PIN_3 | GPIO_PIN_1); //Encender led amariila
            delay(3000); //delay de 3s
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_1, 0); //Apagar led amarilla
            delay(1000); // delay de 1s

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); //Encender led rojo
            delay(3000); //delay de 3s
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); //Apagar led rojo
            bandera = 1; //Encender bandera de terminación
        }
 }
}

void setup(void){
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //Configuración del reloj a 40MHz
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Habilitar el puerto F
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); //Esperar a que se inicialice
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Habilitar el puerto A
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)); //Esperar a que se inicialice
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); //Configurar push1 como entrada
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); //Configurar pines 1, 2 y 3 del Puerto F (LEDS)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5); //Configurar led externo como salida
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //Pull-up al push1
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0); //Iniciar con leds apagados
}

void setupTimer0(void){
    IntMasterEnable(); //Habilitar interrupciones
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Habilitar Timer0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)); //Esperar a que inicialice
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //Configurar como periódico de 32 bits
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 20000000 - 1); //Configurar cada 0.5s
    IntEnable(INT_TIMER0A); //Habilitar interrupcion del timer0 bloque A
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //La interrupción sera cuando se de un timeout (0.5s)
    TimerEnable(TIMER0_BASE, TIMER_A); //Iniciar Timer0
}

void setupUART(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //Habilitar módulo UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); //Definir pinA0 como RX
    GPIOPinConfigure(GPIO_PA1_U0TX); //Definir pinA1 como TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Habilitar los pines para el UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE)); //Setear el UART0 con 115200 de baudrate, de 8 bits, de un dato y sin paridad
    IntEnable(INT_UART0); //Habilitar interrupciones para el UART0
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //Habilitar interrupción para la transmisión y recepción
    UARTEnable(UART0_BASE); //Iniciar UART0
}

void delay(uint32_t msec){
    for (i = 0; i < msec; i++){ //Ejecutar número de veces del argumento
        delay1ms(); //Llamar a función de 1ms
    }
}

void delay1ms(void){
    SysTickDisable(); //Deshabilitar el Sistick
    SysTickPeriodSet(40000-1); //Por aproximadamente 1ms
    SysTickEnable(); //Iniciar el set del periodo

    while((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); //Mientras el bit de count sea 0 no hacer nada
}

void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //Limpiar bandera de interrupción
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) ^ GPIO_PIN_5); //Toggle del LED
}

void UARTIntHandler(void){
    char cReceived; //Variable para guardar dato
    UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT); //Limpiar bandera de interrupción para recepción y transmisión
    while(UARTCharsAvail(UART0_BASE)) //Mientras haya algo disponible en el canal ejecutar
        {
            cReceived = UARTCharGetNonBlocking(UART0_BASE); //Guardar el dato en una variable
            UARTCharPutNonBlocking(UART0_BASE, cReceived); //Devuelve el dato recibido al transmisor
        }
    if (cReceived == 'r'){ //Si se recibió una 'r'
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) ^ GPIO_PIN_1); //Toggle del led rojo
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //Apagar led azul
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
    }

    else if (cReceived == 'g'){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); //Apagar led rojo
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //Apagar led azul
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) ^ GPIO_PIN_3); //Toggle del led verde
    }

    else if (cReceived == 'b'){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); //Apagar del led rojo
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2) ^ GPIO_PIN_2); //Toggle del led azul
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //Apagar led verde
    }
}
