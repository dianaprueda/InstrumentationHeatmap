/* 
 * File:   newmain.c
 * Author: Damian Martinez
 *
 * Created on May 25, 2022, 10:02 PM
 */

// DSPIC33FJ128GP802 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = LPRCDIVN         // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "p33FJ128GP802.h"
#include <pps.h>
#include <string.h>
#include <uart.h>
#include <libpic30.h>



/**********Configuracion Oscilador*****/
#define FCY 7370000/2
#define BAUDRATE 9600
#define BRGVAL ((FCY/BAUDRATE)/16)
#define BRGVAL48 ((FCY/4800)/16)



#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay



/****************************************************/
char* substring(char *destination, const char *source, int beg, int n);
char* getValue(char *data, char separator, int index);
void printString(char *data);
void UART2_Write(char txData);

void SetupIOPins(){
    AD1PCFGLbits.PCFG0 = 1;
    AD1PCFGLbits.PCFG1 = 1;
    
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA4 = 0;
 //   PPSUnLock;  

/* Comunication*/    
   // iPPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP3);      // USB+ RX
    //    iPPSOutput(OUT_PIN_PPS_RP2, OUT_FN_PPS_U1TX);   // USB+ TX   

//    iPPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP10);      // ZBEE RX
//    iPPSOutput(OUT_PIN_PPS_RP11, OUT_FN_PPS_U1TX);   // ZBEE TX 
    
/*  Sensors */       
//    iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP6);      // PH+ RX
//    iPPSOutput(OUT_PIN_PPS_RP7, OUT_FN_PPS_U2TX);   // PH+ TX

//    iPPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP8);      // NO+ RX
//    iPPSOutput(OUT_PIN_PPS_RP9, OUT_FN_PPS_U1TX);   // NO+ TX
//    
    //iPPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP10);      // K+ RX
    //iPPSOutput(OUT_PIN_PPS_RP11, OUT_FN_PPS_U1TX);   // K+ TX
   // PPSLock;  
}

void iniUART(){
    U1MODEbits.STSEL = 0;       // 1-stop bit
    U1MODEbits.PDSEL = 0;       // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0;       // Auto-Baud disabled
    U1MODEbits.BRGH = 0;        // Standard-Speed mode
    U1BRG = BRGVAL;           // Baud Rate setting for 9600
    IEC0bits.U1RXIE = 1;        // Enable RX interrupt
    IPC2bits.U1RXIP = 0b011;    // Uart1 Interrupt priority level=3
    U1STAbits.URXISEL = 1;      // Interrupt after one RX character is received;
    U1MODEbits.UARTEN = 1;      // Enable UART  
    U2STAbits.UTXEN = 1;// Enable UART TX  
}

void iniUART2(){
    U2MODEbits.STSEL = 0;   // 1-stop bit
    U2MODEbits.PDSEL = 0;   // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0;   // Auto-Baud disabled
    U2MODEbits.BRGH = 0;    // Standard-Speed mode
    U2BRG = BRGVAL48;         // Baud Rate setting for 4800
    IEC1bits.U2RXIE = 1;        // Enable RX interrupt
    IPC7bits.U2RXIP = 0b111;    // Uart1 Interrupt priority level=2
    U2STAbits.URXISEL = 1;  // Interrupt after one RX character is received;
    U2MODEbits.UARTEN = 1;  // Enable UART
    U2STAbits.UTXEN = 1;// Enable UART TX    
}

void Init_Timer1( void )
{
    T1CON = 0;              // Timer reset
 	IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
	IPC0bits.T1IP = 6;      // Timer1 Interrupt priority level=4
 	IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
    T1CONbits.TCKPS = 0b10;
 	TMR1=  0x0000;  	
	PR1 = 0xE0EA;           // Timer1 period register = ?????
	T1CONbits.TON = 1;      // Enable Timer1 and start the counter	
}

void SensorPH (void){
    PPSUnLock;      
/*  Sensors */       
  //  iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP6);      // PH+ RX
    iPPSOutput(OUT_PIN_PPS_RP7, OUT_FN_PPS_U2TX);   // PH+ TX
    iPPSOutput(OUT_PIN_PPS_RP9, 0x0000);   // NO+ TX --- OFF
    iPPSOutput(OUT_PIN_PPS_RP11, 0x0000);   // K+ TX --- OFF
    PPSLock;  
}
void SensorNO (void){
    PPSUnLock;      
/*  Sensors */       
    iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP8);      // NO+ RX
    iPPSOutput(OUT_PIN_PPS_RP11, 0x0000);
    iPPSOutput(OUT_PIN_PPS_RP7, 0x0000);   // PH+ TX
    iPPSOutput(OUT_PIN_PPS_RP9, OUT_FN_PPS_U2TX);   // NO+ TX
       // K+ TX --- OFF
    PPSLock;  
}

void SensorK (void){
    PPSUnLock;      
/*  Sensors */       
//    iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP3);      // K+ RX
//    iPPSOutput(OUT_PIN_PPS_RP2, OUT_FN_PPS_U2TX);   // K+ TX
    iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP10);      // K+ RX
    //iPPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP12);      // ZBEE RX
   
    iPPSOutput(OUT_PIN_PPS_RP11, OUT_FN_PPS_U2TX); 
 //   iPPSOutput(OUT_PIN_PPS_RP13, OUT_FN_PPS_U2TX); 
    iPPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP3);      // USB+ RX
        iPPSOutput(OUT_PIN_PPS_RP2, OUT_FN_PPS_U1TX);   // USB+ TX   
    
    iPPSOutput(OUT_PIN_PPS_RP7, 0x0000);   // PH+ TX
    iPPSOutput(OUT_PIN_PPS_RP9, 0x0000);   // K+ TX --- OFF
    PPSLock;  
}
    unsigned int Count;
    unsigned int estado=0;
    unsigned int stall=0;
    
    char ISFET_String[32]= "*";  // ISFET interface response
    int ISFET_IN = 0;
    char COMMANDS[15]= "*";  // ISFET interface response
    char Xbee_ON[] = "XON\n";
    char Xbee_OFF[] = "XOFF\n";
    const char *PXbeeOn = Xbee_ON;
    const char *PXbeeOff = Xbee_OFF;
    
    int COMMANDS_IN = 0;
    int check = 0;          // counter for data transfer check 
    unsigned int FlagTx = 0;
    unsigned int FlagCmdTx = 0;
    char msg[]="data\n";
    int pH_mV;
    int pH;
    
    int FlagXbee = 1;
    
    
int main(int argc, char** argv) {
    AD1PCFGL = 0xFFFF;                  // Set all pins to digital
    SetupIOPins();
    //__C30_UART=2;
   // SensorK();
    iniUART();
    iniUART2();
    for(int i=0; i<200; i++)
          Nop();
    Init_Timer1();
    PORTAbits.RA4= 1;
    
    while(1){
        Idle();
        PORTAbits.RA1= 1;
        
        
        char *data;

        /* Check for receive errors */
//        if(U1STAbits.FERR == 1){
//            continue;
//        }
        if(U2STAbits.FERR == 1){
           continue;
        }
    
        /* Must clear the overrun error to keep UART receiving */
        if(U1STAbits.OERR == 1){
            U1STAbits.OERR = 0;
            continue;
        }
        
        if(U2STAbits.OERR == 1){
            U2STAbits.OERR = 0;
            continue;
        }
        
          if (FlagCmdTx== 1){
              
              if (strcmp (COMMANDS ,PXbeeOn )== 0){
                  PPSUnLock;  
                  iPPSOutput(OUT_PIN_PPS_RP13, OUT_FN_PPS_U1TX);    // ZBEE TX 
         
                  PPSLock; 
                  stall=0;
                  PORTAbits.RA4= 0;
                  FlagXbee =0;
                  
              };
              
              if (strcmp (COMMANDS ,&Xbee_OFF )== 0){
                  PPSUnLock;  
                  iPPSOutput(OUT_PIN_PPS_RP13, 0x0000);    // ZBEE TX 
                  PPSLock;  
                  PORTAbits.RA4= 1;
                  FlagXbee =1;
              };      
              
              
              
              
            for(int i=0; i < sizeof(COMMANDS); i++)   //empty ISFET_Serial buffer
                COMMANDS[i] = (char)0;  
              COMMANDS_IN = 0;
               FlagCmdTx=0;
          }
        
        
        
        
        
        
        
        if (FlagTx== 1){

         if(ISFET_String[0] == 'd'){
            data = getValue(ISFET_String,';',3);
            int checkIN = atoi(data);
     
//            if(checkIN == (check - strlen(data))){
                data = getValue(ISFET_String,';',1);
                pH_mV = (int) atoi(data);
                data = getValue(ISFET_String,';',2);
                pH = (int) atoi(data);
             printf("Data: ");
                printf("%d",pH_mV);      //print raw data
                printf("mV - pH ");
                printf("%.2f",(double) pH/100);  //print calculated pH
                printf("\n");
 //           }
        }
        else{
//            printf("Check conecction\n");
        }
        FlagTx = 0;
        ISFET_IN = 0;
        check = 0;          // counter for data transfer check 
        
        for(int i=0; i < sizeof(ISFET_String); i++)   //empty ISFET_Serial buffer
            ISFET_String[i] = (char)0;   
    
        
    }        
        
 
        
        
        
        switch (estado){
        
            case 2: // Sensor de PH
                 estado=0;
                //SensorPH();
                
                  stall=2;
                //  printString(msg);
        
                  printf("PH: \n");
                             
                // putsUART1((unsigned int *)msg);
                  SensorNO();
                  
                  
                  
            break;
    
            case 4: // Sensor de NO3
                estado=0;
                stall=4;
               // SensorNO();
                                  
                printf("NO3: \n");
                        
                 printString(msg);
                    
                if (Count ==8)
                 SensorK();
     
            break;
            
            case 6: // Sensor de K+
                estado=0;
               // SensorK();
                  stall=6;
           //     DELAY_105uS;
                           
               printf("K+: \n");

               
                // printf("data\n");
                printString(msg);
                 
              if (Count ==12)  
                SensorPH();
                
               // putsUART1((unsigned int *)msg);
            break;              
        }
                
        
    }
    return (EXIT_SUCCESS);
}

/* ISR ROUTINE FOR THE TIMER1 INTERRUPT */

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt( void )
{
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    if (Count < 6)
        estado = Count;
   
    if (Count >= 4 && Count<8)
        estado = 4;
    if (Count >= 8 && Count<12)
        estado = 6;
     if (Count >=12)
        Count = 0;
        
//    
       Count++;
////    printf ("%d", Count);
//    
//    
//	if(Count > 5 && C){
//        Count= 6;  
//        LATAbits.LATA0 = ~LATAbits.LATA0;
//	}
//    
   
    TMR1          = 0;
	T1CONbits.TON = 1;
	/* reset Timer 1 interrupt flag */
}


void __attribute__((__interrupt__,no_auto_psv)) _U1RXInterrupt(void)
{
    while(U1STAbits.URXDA == 1){
       // printf("Error\n");
        char c = U1RXREG;
        COMMANDS[COMMANDS_IN] = c;
        COMMANDS_IN++;
//        check ++;            
        if(c == '\n'){
            FlagCmdTx = 1;
        }
        IFS0bits.U1RXIF = 0;
    }
    IFS0bits.U1RXIF = 0;
}


void __attribute__((__interrupt__,no_auto_psv)) _U2RXInterrupt(void)
{
    while(U2STAbits.URXDA == 1){
       
        char c = U2RXREG;
        ISFET_String[ISFET_IN] = c;
        ISFET_IN++;
        check ++;            
        if(c == '\n'){
            FlagTx = 1;
        }
    }
    IFS1bits.U2RXIF = 0;
}



char* substring(char *destination, const char *source, int beg, int n){
    // extracts `n` characters from the source string starting from `beg` index
    // and copy them into the destination string
    while (n > 0)
    {
        *destination = *(source + beg);
 
        destination++;
        source++;
        n--;
    }
 
    // null terminate destination string
  //  *destination = '\0';
     *destination = '\n';
 
    // return the destination string
    return destination;
}

char* getValue(char *data, char separator, int index){
    
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = strlen(data) - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data[i] == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    static char destination[10];
    if (found > index ){
        substring(destination, data, strIndex[0], strIndex[1]-strIndex[0] );
        return destination;
    }
    else{
        return "";
    }
}

void printString(char *data)
{
     
  // char casa[]="ladrar\n";
  // putsUART2((unsigned int *)casa);
  
    int len = strlen(data);
    int i =0;
    for( i = 0; i<len; i++)
    {
        char txData = *(data + i);
        UART2_Write(txData);
    }
}

void UART2_Write(char txData)
{

    while(U2STAbits.UTXBF == 1)
    {
    }
    U2TXREG = txData;    // Write the data byte to the USART.
}


