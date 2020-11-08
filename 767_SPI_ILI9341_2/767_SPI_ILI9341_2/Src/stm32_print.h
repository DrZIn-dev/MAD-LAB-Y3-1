#include "usart.h"
#include "string.h"

extern UART_HandleTypeDef huart3;

struct _Serial {
    void (*print)(char *str);
    void (*println)(char *str);
    void (*printc)(char ch);
    char (*getch)();
};

void h3_print(char *str){
    while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),1000);
}

void h3_println(char *str){
    while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),1000);
    h3_print("\r\n");
}

void h3_printc(char ch){
    while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1,1000);
}

char h3_getch(){
    char ch1;
	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE)== RESET){}
	HAL_UART_Receive(&huart3, (uint8_t*) &ch1, 1, 1000);
	return ch1;
}

struct _Serial Serial = (struct _Serial){.print=h3_print, .println=h3_println, .printc=h3_printc, .getch=h3_getch};

void print(UART_HandleTypeDef *_huart, char *str){
    while(__HAL_UART_GET_FLAG(_huart,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(_huart, (uint8_t*) str, strlen(str),1000);
}

void println(UART_HandleTypeDef *_huart, char *str){
    while(__HAL_UART_GET_FLAG(_huart,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(_huart, (uint8_t*) str, strlen(str),1000);
    print(_huart,"\r\n");
}

void printc(UART_HandleTypeDef *_huart, char ch){
    while(__HAL_UART_GET_FLAG(_huart,UART_FLAG_TC)==RESET){}
    HAL_UART_Transmit(_huart, (uint8_t*) &ch, 1,1000);
}

char getch(UART_HandleTypeDef *_huart){
    char ch1;
	while(__HAL_UART_GET_FLAG(_huart,UART_FLAG_RXNE)== RESET){}
	HAL_UART_Receive(_huart, (uint8_t*) &ch1, 1, 1000);
	return ch1;
}