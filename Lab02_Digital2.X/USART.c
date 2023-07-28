/* Archivo: USART.c
 * Dispositivo: PIC16F887
 * Autor: Kevin Alarcón
 * Compilador: XC8(v2.40), MPLABX V6.05
 * 
 * 
 * Programa: Librería de configuracion para comunicación serial
 * Hardware: NA
 * 
 * Creado: 17 de julio, 2023
 * Última modificación: 28 de julio, 2023
 */

#include "USART.h"

void UART_RX_config (uint16_t baudrate){
    //paso 1
    SPBRGH = 0; //%error = (9615.38-9600)/9600 * 100 = 0.16%
    BRG16 = 0;
    
    //paso 2
    CREN = 1;
    SPEN = 1;   //RCSTAbits ? apagar bandera TX
    
    //paso 3: habilitar los 9 bits
    RCSTAbits.RX9 = 0;
    
    //paso 5: interrupciones
    PIE1bits.RCIE = 1; //Habilitamos interrupción de RCIF
    PIR1bits.RCIF = 0; //Limpiamos bandera del RCIF
    
    //Dependediendo de la velocidad de transmisión que se desee se le asignará un número
    //diferente al SPBRG
    if(baudrate == 1200){
        SPBRG = 103;
    }
    else if(baudrate == 2400){
        SPBRG = 51;
    }
    else if(baudrate == 9600){
        SPBRG = 12;
    }
    else if(baudrate == 10417){
        SPBRG = 11;
    }
    return;
}

void UART_TX_config (uint16_t baudrate){
    TXSTAbits.BRGH = 0;   
    BRG16 = 0;
    TXSTAbits.SYNC = 0;   // TXSTAbits ? habilitar transmision & TXIF = 1
    TXSTAbits.TXEN = 1; //permite transmitir
    TXIF = 0; //PIRbits -> apagar bandera TX
    
    //Dependediendo de la velocidad de transmisión que se desee se le asignará un número
    //diferente al SPBRG
    if(baudrate == 1200){
        SPBRG = 103;
    }
    else if(baudrate == 2400){
        SPBRG = 51;
    }
    else if(baudrate == 9600){
        SPBRG = 12;
    }
    else if(baudrate == 10417){
        SPBRG = 11;
    }
    return;
}

void UART_write_char(char txt[]){
    int i;  //variable iteracion
    for(i = 0; txt[i] != '\0'; i++){
        
        while(!PIR1bits.TXIF); // Esperamos a que el registro de transmisión esté vacío
        //TXREG es el registro de la terminal, es decir, lo que posea ese registro se va a presentar en la terminal
        TXREG = txt[i]; // Envia caracter por caracter a la terminal
    }
    return;
}

char UART_read_char (){
    while(!PIR1bits.RCIF); //Esperar a que presione un valor en la terminal
    char CONT[15]; //Creamos un arreglo
    char caracter = RCREG; //Le ingresamos el valor ingresado de la terminal a la variable caracter 
    if (caracter == '+'){ //Si caracter es un +
        PORTB++; //Incrementamos el puertoB
    }
    else if (caracter == '-'){//Si caracter es un -
        PORTB--; //Decrementamos el puertoB
    }
    sprintf(CONT, "%d\r", PORTB); //Pasa a el valor del puerto B a caracter en la variable CONT
    return CONT; //Retornamos éste caracter
}