/*
 * File:   main.c
 * Author: Pablo
 * Ejemplo de uso de la LCD 16x2 en modo 4 bits
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Created on 31 de enero de 2020, 11:20 AM
 */

#pragma config FOSC = EXTRC_NOCLKOUT // Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
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
#include "LCD.h"

//---------------------Constantes---------------------------------
#define _XTAL_FREQ 8000000
#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RB5
#define D6 RD6
#define D7 RD7

//---------------------Variables---------------------------------
int numero = 0;
char CVPOT[4]; //Creamos un arreglo

//------------------Prototipos-----------------------------------
void POT (void);
void setup(void);

//----------------------Interrupciones---------------------------
void __interrupt() isr(void) {
    //Interrupción del ADC
    if (PIR1bits.ADIF) {
        numero = adc_read(); //Le asignamos a número el valor del ADC
        PIR1bits.ADIF = 0; //Limpiar la bandera de la interrupcion del ADC
    }
    
    return;
}

void main(void) {
    setup();
    Lcd_Clear();
    unsigned int a;
    Lcd_Init();
    while(1)
    {
        POT();
        PORTB = numero;
        //Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("Valor");
        Lcd_Set_Cursor(2,1);
        Lcd_Write_String(CVPOT);
        //__delay_ms(00);
        //Lcd_Clear();
        /*Lcd_Set_Cursor(1,1);
        Lcd_Write_String("Developed By");
        Lcd_Set_Cursor(2,1);
        Lcd_Write_String("electroSome");
        __delay_ms(2000);
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("www.electroSome.com");

        for(a=0;a<15;a++)
        {
            __delay_ms(300);
            Lcd_Shift_Left();
        }

        for(a=0;a<15;a++)
        {
            __delay_ms(300);
            Lcd_Shift_Right();
        }

        Lcd_Clear();
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char('H');
        Lcd_Write_Char('o');
        Lcd_Write_Char('l');
        Lcd_Write_Char('a');
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("Hola Mundo");
        __delay_ms(2000);*/

        if (ADCON0bits.GO == 0) { // Si la lectura del ADC se desactiva

            __delay_us(1000); //Este es el tiempo que se dará cada vez que se desactiva la lectura
            ADCON0bits.GO = 1; //Activamos la lectura del ADC
        }
    }
    return;
}
  

void POT (void)
{       ADON = 1; //se enciende el ADC
        __delay_ms(10);
        ADCON0bits.GO = 1; //Empieza la conversion
     
        sprintf(CVPOT, "%d\r", numero); //Pasa a el valor del potenciometro a caracter
        ADON = 0;       //se apaga el ADC
        return;      
}

void setup(void){
    adc_init(0);
    //definir digitales
    ANSELbits.ANS0 = 1; //Seleccionamos solo los dos pines que utilizaremos como analógicos
    ANSELH = 0; 
    
    //Definimos puertos que serán entradas
    TRISA = 0b11111111;
    
    //Definimos puertos que serán salidas
    TRISB = 0;
    TRISD = 0;
    TRISE = 0;
    
    //Limpiamos los puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    OSCCONbits.IRCF = 0b111; //8 MHz
    return;
}