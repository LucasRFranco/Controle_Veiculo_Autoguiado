//----------
// Bibliotecas
#include <stdbool.h>
#include <stdio.h>
#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"?
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"


//----------
// RTOS
osThreadId_t protocolo_id, obstaculo_id, trajeto_id;
osMutexId_t uart_id;

const osThreadAttr_t protocolo_attr = {
  .priority = osPriorityNormal
};

const osThreadAttr_t trajeto_attr = {
  .priority = osPriorityAboveNormal
};

const osThreadAttr_t obstaculo_attr = {
  .priority = osPriorityHigh
};


//----------
// Variáveis globais
int32_t Laser = 0;
int32_t Camera = 0;


//----------
// Declaração de Funções
int Leitura(int sensor);


//----------
// UART definitions
extern void UARTStdioIntHandler(void);

void UARTInit(void){
  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 9600, SystemCoreClock);

  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
} // UARTInit

void UART0_Handler(void){
  UARTStdioIntHandler();
} // UART0_Handler


//----------
// Threads
void protocolo(void *arg){
  int32_t Ll = 0;
  int32_t Lbcam = 0;
  uint32_t tick;
  
  while(1){
    tick = osKernelGetTickCount();
    
    // Leitura Laser
    osMutexAcquire(uart_id, osWaitForever);
    UARTprintf("Pl;"); // Solicita Leitura do Sensor a Laser
    //Ll = Leitura(1);
    Ll = -1;
    osMutexRelease(uart_id);
    
    // Flag Obstáculo
    if(Ll != -1){
      Laser = Ll;
      osThreadFlagsSet(obstaculo_id, 0x0001);
    }
    
    osDelayUntil(tick + 500);
    tick = osKernelGetTickCount();
    
    // Leitura Câmera
    osMutexAcquire(uart_id, osWaitForever);
    UARTprintf("Pbcam;"); // Solicita Leitura da Câmera
    //Lbcam = Leitura(2);
    Lbcam = 0;
    osMutexRelease(uart_id);
    
    // Flag trajeto
    if((Lbcam < -25) || (Lbcam > 25)){
      Camera = Lbcam;
      osThreadFlagsSet(trajeto_id, 0x0001);
    }
    
    osDelayUntil(tick + 500);
  } // while
} // protocolo

void obstaculo(void *arg){
  int32_t Lu = 0;
  uint32_t tick;
  
  while(1){
    // Aguarda Flag Obstáculo
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    
    //Desvio
    osMutexAcquire(uart_id, osWaitForever);
    UARTprintf("A+2;");
    UARTprintf("V+45;");
    osMutexRelease(uart_id);
    
    // Leitura Ultrassom
    osMutexAcquire(uart_id, osWaitForever);
    while(Lu != -1){
      tick = osKernelGetTickCount();
      UARTprintf("Pu;"); // Solicita Leitura do Sensor Ultrassônico
      osDelayUntil(tick + 200);
      //Lu = Leitura(1);
      Lu = -1;
     }
    osMutexRelease(uart_id);
    
    // Retoma Posição
    osMutexAcquire(uart_id, osWaitForever);
    UARTprintf("V-45;");
    UARTprintf("A+5;");
    osMutexRelease(uart_id);
  } // while
} // obstaculo

void trajeto(void *arg){
  int32_t A = 0;
  int32_t V = 0;
  
  while(1){
    // Aguarda Flag Trajeto
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    
    // Detecta Faixa da Pista
    if(Camera < 200){
      if(Camera < 150){
        if(Camera < 100){
          if(Camera < 50){
            if(Camera <= -25){
              if(Camera <= -50){
                if(Camera <= -100){
                  if(Camera <= -150){
                    if(Camera <= -200){
                      osMutexAcquire(uart_id, osWaitForever);
                      UARTprintf("S;"); // Para o Carro
                      osMutexRelease(uart_id);
                    }
                    else{
                      A = - 4;
                      V = + 90;
                    }
                  }
                  else{
                    A = - 3;
                    V = + 60;
                  }
                }
                else{
                  A = - 2;
                  V = + 45;
                }
              }
              else{
                A = - 1;
                V = + 30;
              }
            }
            else{
              A = - 1;
              V = - 30;
            }
          }
          else{
            A = - 2;
            V = - 45;
          }
        }
        else{
          A = - 3;
          V = - 60;
        }
      }
      else{
        A = - 4;
        V = - 90;
      }
    }
    else{
      osMutexAcquire(uart_id, osWaitForever);
      UARTprintf("S;"); // Para o Carro
      osMutexRelease(uart_id);
    }
    
    // UART TX
    if((Camera < 200) && (Camera > -200)){
      osMutexAcquire(uart_id, osWaitForever);
      UARTprintf("A%i;", A); // Freia
      UARTprintf("V%i;", V); // Corrige Posição
      UARTprintf("A+5;"); // Acelera
      osMutexRelease(uart_id);
    }

  } // while
} // trajeto


//----------
// Functions
void Start(void){
  protocolo_id = osThreadNew(protocolo, NULL, &protocolo_attr);
  obstaculo_id = osThreadNew(obstaculo, NULL, &obstaculo_attr);
  trajeto_id   = osThreadNew(trajeto  , NULL, &trajeto_attr);
  uart_id = osMutexNew(NULL);
  
  UARTprintf("A+5;"); // Aceleração = 5m/s^2
  
} // Start


int Leitura(int sensor){
  int init = 0;
  int dot = 0;
  int i = 0;
  int valor = 0;
  char buffer[10];
  char convert[5];
  
  
  for(i = 0; i < 10; i++)
  {
    buffer[i] = 0;
  }
  
  init = UARTPeek((unsigned char)"L");
  dot  = UARTPeek((unsigned char)".");
  
  for(i = init; i < dot; i ++)
  {
    buffer[i] = UARTgetc();
  }
  
  switch(sensor){
  case 1: //Laser ou Ultrassom
    for(i = 0; i < 2; i++)
    {
      convert[i] = buffer[i+2];
    }
    break;
  case 2: // Câmera
    for(i = 0; i < 5; i++)
    {
      convert[i] = buffer[i+5];
    }
    break;
  }
  
  valor = (int)convert;
  
  return valor;
  
} // Leitura


//----------
// Main
void main(void){
  
  UARTInit();
  UARTFlushTx(1);
  
  if(osKernelGetState() == osKernelInactive)
     osKernelInitialize();
  
  // Criar Threads e Mutex
  // Colocar carro em movimento
  Start();
  
  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
} // main