/* Archivo: Lab02.c
 * Dispositivo: PIC16F887
 * Autor: Kevin Alarcón
 * Compilador: XC8(v2.40), MPLABX V6.05
 * 
 * 
 * Programa: Realizar dos contadores, uno por medio de comunicación serial y otro con ADC a través de librerías para mostrarlos en
 * //una pantalla LCD
 * Hardware: Potenciometros, leds, pantalla LCD y un FTDI
 * 
 * Creado: 25 de julio, 2023
 * Última modificación: 28 de julio, 2023
 */

#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include "ADC.h"
#include "LCD8bits.h"
#include "USART.h"

//---------------------Constantes---------------------------------
#define _XTAL_FREQ 8000000
#define RS RC0
#define EN RC1
#define D4 RD4
#define D5 RB5
#define D6 RD6
#define D7 RD7

//---------------------Variables---------------------------------
int numero = 0;
char CVPOT[15]; //Creamos un arreglo
char CONT; //Creamos un arreglo
int voltaje = 0;
int opcion = 0;
uint16_t contador = 0;
char menu[] =  "Menu: \r 1)Leer potenciometro \r 2)Incrementar/decrementar puerto \r";

//------------------Prototipos-----------------------------------
void POT (void);
void setup(void);

//----------------------Interrupciones---------------------------
void __interrupt() isr(void) {
    if(PIR1bits.RCIF){ //Verificamos que la bandera del EUSART esté llena (ya recibió un valor)
        opcion = RCREG; //Ingresamos el dato recibido desde la hiperterminal en la variable opcion
        //RCREG es el registro que contiene el valor que ingresamos en la hiperterminal
        switch(opcion){ //Hacemos una comprobación de la variable opcion
            case '1': //Si eligieron la opcion 1
                UART_write_char("El valor del potenciometro es:");
                UART_write_char(CVPOT); //Mostramos en la terminal el valor del pot en voltaje
                
                UART_write_char(menu);//Muestra el menu de nuevo
                break; 
            case '2':  //Si eligieron la opcion 2
                while(RCREG != 32){ //No sale del bucle hasta que opriman space en la hiperterminal
                    CONT = UART_read_char();  //Mandamos a llamar a nuestras funciones
                }
                UART_write_char(menu); //Muestra el menu de nuevo
                break;
            default: //Si ingresa un valor diferente
                
                UART_write_char("No existe esa opcion \r"); //
                while(RCREG != 32); //No sale del bucle hasta que opriman space en la hiperterminal
                UART_write_char(menu); //Muestra el menu de nuevo
        }   

    }
    
    //Interrupción del ADC
    if (PIR1bits.ADIF) {
        numero = adc_read(); //Le asignamos a número el valor del ADC
        PIR1bits.ADIF = 0; //Limpiar la bandera de la interrupcion del ADC
    }
    
    return;
}

void main(void) {
    setup();
    Lcd_Init8(); //Inicializamos la LCD
    Lcd_Clear8(); //Limpiamos la LCD
    unsigned int a;
    
    while(1)
    {
        
        POT(); 
        Lcd_Set_Cursor8(1,1); //Definimos en donde queremos ver Volt en la LCD
        Lcd_Write_String8("Volt");
        Lcd_Set_Cursor8(2,1); //Definimos en donde queremos ver el voltaje en la LCD
        Lcd_Write_String8(CVPOT);
        Lcd_Set_Cursor8(1,8); //Definimos en donde queremos ver Cont en la LCD
        Lcd_Write_String8("Cont");
        Lcd_Set_Cursor8(2,9); //Definimos en donde queremos ver el contador en la LCD
        Lcd_Write_String8(CONT);

        if (ADCON0bits.GO == 0) { // Si la lectura del ADC se desactiva

            __delay_us(1000); //Este es el tiempo que se dará cada vez que se desactiva la lectura
            ADCON0bits.GO = 1; //Activamos la lectura del ADC
        }
            
        if (PIR1bits.RCIF == 0){ //Espera a que la persona ingrese una opcion
            __delay_ms(500);
        }

    }
    return;
}
  

void POT (void)
{       ADON = 1; //se enciende el ADC
        __delay_ms(10);
        ADCON0bits.GO = 1; //Empieza la conversion
        voltaje = map(numero, 0, 255, 0, 5);
        sprintf(CVPOT, "%d\r", voltaje); //Pasa a el valor del potenciometro a caracter
        ADON = 0;       //se apaga el ADC
        return;      
}

void setup(void){
    adc_init(0);
    UART_RX_config(9600); //Configuración de los registros para la comunicación serial
    UART_TX_config(9600);
    //definir digitales
    ANSELbits.ANS0 = 1; //Seleccionamos solo los dos pines que utilizaremos como analógicos
    ANSELH = 0; 
    
    //Definimos puertos que serán entradas
    TRISA = 0b11111111;
    TRISC = 0b11000000;
    
    //Definimos puertos que serán salidas
    TRISB = 0;
    TRISD = 0;
    
    //Limpiamos los puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    OSCCONbits.IRCF = 0b111; //8 MHz
    return;
}

