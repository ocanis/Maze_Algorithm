/* ************************************************************************** */
#include "mbed.h"
#include "m3pi.h"

#include <string.h>
/* ************************************************************************** */
#define MAXSAVE 4000

#define MAX_SPEED 0.3
#define MIN_SPEED -0.2

#define P_TERM  0.9
#define I_TERM  0.1
#define D_TERM 0.0

#define SPEED 0.3
/* ************************************************************************** */
#define seuil(x) (x>350 ? 1 : 0)
/* ************************************************************************** */
char explorer (void);
unsigned char  lecture_captsol (unsigned short *tab);
char rejouer (void);
void PIDf (void);
void simplifier (char *T);
/* ************************************************************************** */
BusOut leds(LED1, LED2, LED3, LED4);
LocalFileSystem local("local");
m3pi m3pi;                                  // Initialise the m3pi
Ticker tk10ms;

volatile char flag10ms;

unsigned short tabsensor[5];
unsigned char statcapt;
unsigned char LFRstat, LFRvect;
unsigned char sauvetab[MAXSAVE][3];

char texttab[100];
//unsigned char TXT[20]={"FLFLLRLLRR"};
unsigned short intercount,index;

unsigned lvwait;
char mess[20];
/* ************************************************************************** */
void isr10ms (void) {
        flag10ms = 1;
}
/* ************************************************************************** */
char explorer (void) {
    static unsigned char etatExplorer = 0;
    
    switch (etatExplorer) {
        case 0 :
            PIDf(); 
            switch (LFRvect) {
                case 0x77 :
                    etatExplorer = 37;
                    lvwait = 0;
                    break;
                    
                case 0x00 :
                    //m3pi.stop();
                    etatExplorer = 3;
                    lvwait = 0;
                    texttab[intercount++] = 'B';
                    break;
                
                case 0x33 :
                    etatExplorer = 13;
                    texttab[intercount++] = 'R';
                    break;
                
                case 0x66 :
                    etatExplorer = 26;
                    break;
    
                case 0x22 :
                default :
                    break;
            }
            break;
            
        case 3 :
            lvwait++;
            if (lvwait > (60/10)) {
                m3pi.stop();
                etatExplorer = 1;
            }
            break;
        
        case 1 : //lancer STOP avec rotation  à droite
            lvwait++;
            if (lvwait > (500/10)) {
                m3pi.right(0.2);
                etatExplorer = 2;
            }
            break;
        
        case 2 : // jattends pour STOPPER
            if (LFRvect == 0x22) {
                m3pi.stop();
                etatExplorer = 0;
            }
            break;
             
        case 13 : //inter droite ou tout droit 
            PIDf();  
            switch (LFRvect) {
                case 0x00 :
                    m3pi.stop();
                    lvwait = 0;
                    etatExplorer = 1;  
                    break;
                    
                case 0x22 :
                    //m3pi.stop();
                    lvwait = 0;
                    etatExplorer = 14;  
                    break;
                    
                default :
                    break;
            }
            break; 

        case 14 : 
            lvwait++;
            if (lvwait > (60/10)) {
                m3pi.stop();
                etatExplorer = 15;
                lvwait = 0;
            }
            break;

        case 15 :
            lvwait++;
            if (lvwait > (500/10)) {
                m3pi.right(0.2);
                etatExplorer = 16;
                lvwait = 0;
            }
            break;
        
        case 16 :        
            if ((LFRvect&0x22) == 0x00) {
                etatExplorer = 2;
            }
            break;
        
        case 26 : // inter  gauche ou tout droit
            PIDf();  
            switch (LFRvect) {
                case 0x00 :
                    //m3pi.stop();
                    lvwait = 0;
                    etatExplorer = 27; 
                    texttab[intercount++] = 'L';
                    break; 
                
                case 0x22 :
                    texttab[intercount++] = 'F';
                    etatExplorer = 0; 
                    break; 
                
                default :
                    break;
            }
            break; 
        
        case 27 :
            lvwait++;
            if (lvwait > (50/10)) {
                m3pi.stop();
                etatExplorer = 28;
                lvwait = 0;
            } 
            break;      
        
        case 28 :
            lvwait++;
            if (lvwait > (500/10)) {
                m3pi.left(0.2);     
                etatExplorer = 2;
            }
            break;
        
        case 37 :
            PIDf(); 
            lvwait++;
            if (LFRvect == 0x77) {
                if (lvwait > (150/10)) {
                    m3pi.stop();
                    etatExplorer = 38;
                }
            }
            else {
                texttab[intercount++] = 'R';
                etatExplorer = 13;
                lvwait = 0;    
            }
            break;
        
        case 38 :
            leds = 0x0F;
            return 1;
            break;
        
        default :
            break;
    }
    
    return 0;                  
}
/* ************************************************************************** */
unsigned char  lecture_captsol (unsigned short *tab) {
    unsigned char stat = 0;
    
    m3pi.calibrated_sensors(tab);
    for(unsigned short i=0; i < 5; i++) stat = (stat<<1) | seuil(tab[i]);
    LFRstat = (((stat&0x04) == 0x04) || ((stat&0x03) == 0x02) || ((stat&0x18) == 0x08) ? 0x02 : 0) | (((stat&0x03) == 0x03) ? 0x01 : 0) | (((stat&0x18) == 0x18) ? 0x04 : 0);
    
    return stat;
}
/* ************************************************************************** */
void PIDf (void) {
    float current_pos_of_line, derivative, left, power, proportional, right;
    static float integral = 0.0, previous_pos_of_line = 0.0;
    
    // Get the position of the line.
    current_pos_of_line = m3pi.line_position();
            
    proportional = current_pos_of_line;    
    // Compute the derivative
    derivative = current_pos_of_line - previous_pos_of_line;
    // Compute the integral
    integral = (integral + I_TERM*proportional) / (1+I_TERM);
    
    // Remember the last position
    previous_pos_of_line = current_pos_of_line;
    
    // Compute the power
    power = (proportional*(P_TERM)) + (integral*(I_TERM)) + (derivative*(D_TERM));
    // Compute new speeds
    right = SPEED - (power*MAX_SPEED);
    left = SPEED + (power*MAX_SPEED);
    // Limit checks on motor control
    left = (left > MAX_SPEED ? MAX_SPEED : (left < MIN_SPEED ? MIN_SPEED : left));
    right = (right > MAX_SPEED ? MAX_SPEED : (right < MIN_SPEED ? MIN_SPEED : right));
    // Send command to motors
    m3pi.left_motor(left);
    m3pi.right_motor(right);
}
/* ************************************************************************** */
char rejouer (void) {
    static unsigned char etatRejouer = 0;
    
    switch (etatRejouer) {
        case 0 :
            PIDf();
            switch (LFRvect)
            {
                case 0x77 :
                    etatRejouer = 37;
                    lvwait = 0;
                    break;
                    
                case 0x00 : // pas bon
                    m3pi.stop();
                    m3pi.cls();
                    sprintf(mess, "ERR0:%u", index);
                    m3pi.print(mess, strlen(mess));
                    while(1);
                    break;

                case 0x33 :
                    etatRejouer = 13;
                    break;

                case 0x66 :
                    etatRejouer = 13;
                    break;

                case 0x22 :
                default :
                    break;
            }          
            break;

        case 2 : // fin rotation droite
            if (LFRvect == 0x22) {
                m3pi.stop();
                etatRejouer = 0;
            }
            break; 
        
        case 13 : //inter droite ou tout droit
            PIDf(); 
            if((LFRvect == 0x00) || (LFRvect == 0x22)) {
                switch (texttab[index]) {
                    case 'F' :
                        index++;
                        etatRejouer = 0;
                        break;
                    
                    case 'R' :
                        //m3pi.stop();
                        lvwait = 0;
                        index++;
                        if (LFRvect == 0x00) {
                            m3pi.right(0.2);
                            etatRejouer = 2;
                        }
                        else {
                            etatRejouer = 14;  //on tourne à droite
                        }
                        break;

                    case 'L' :
                        lvwait = 0;
                        index++;
                        if (LFRvect == 0x00) {
                            m3pi.left(0.2);
                            etatRejouer = 2;
                        }
                        else {
                            etatRejouer = 17;  //on tourne à droite
                        }
                        break;
                    
                    default :
                        break;
                }
            }
            break;

        case 14 :
            lvwait++;
            if (lvwait > (60/10)) {
                m3pi.stop();
                etatRejouer = 15;
                lvwait = 0;
            }
            break;

        case 15 :
            lvwait++;
            if (lvwait > (500/10)) {
                m3pi.right(0.2);
                etatRejouer = 16;
                lvwait = 0;
            }
            break;
        
        case 16 :
            if ((LFRvect&0x22) == 0x00) {
                etatRejouer = 2;
            }
            break;
            
        case 17 :
            lvwait++;
            if (lvwait > (60/10)) {
                m3pi.stop();
                etatRejouer = 18;
                lvwait = 0;
            }
            break;
        
        case 18 :
            lvwait++;
            if (lvwait > (500/10)) {
                m3pi.left(0.2);
                etatRejouer = 16;
                lvwait = 0;
            }
            break;
            
        case 37 :
            PIDf();
            lvwait++;
            if (LFRvect == 0x77) {
                if (lvwait > (150/10)) {
                    m3pi.stop();
                    etatRejouer = 38;
                }
            }
            else {
                etatRejouer = 13;
                lvwait = 0;   
            }
            break;
        
        case 38 :
            leds = 0x0F;
            return 1;
            break;
            
        default :
            break;
    }
    
    return 0;
}
/* ************************************************************************** */
char *ppg, *ppi, *ppd;

void simplifier (char *T) {
   
    short fl = 1;
    unsigned m, strlenT;
    
    while (strchr((char*)T, 0x42))
    {
        ppi = (char*)strchr((char *)T, 0x42);
        ppg = ppi-1;
        ppd = ppi+1;
        strlenT=strlen(T);
        while (fl == 1) 
        {
            m = (((unsigned)(*ppg))<<8) | (unsigned)(*ppd);
            switch (m){
                case 0x524C :
                case 0x4C52 :
                            if((unsigned)(ppg-1) < (unsigned)((char *)T)) ppg=(char *)T;
                            else ppg--;
                            
                            if((unsigned)(ppd+1) > (unsigned)(T+strlen(T)-1)) ppd=((char *)T+strlen(T)-1);
                            else ppd++;
                            break;

                case 0x5252 :
                            if((unsigned)(ppd+1) < (unsigned)(T+strlen(T)-1)) {
                                *ppg = 'F';
                                ppg++;
                                *ppg = 0;
                                strcat((char *)ppg, (char*)ppd+1);
                            }
                            else {
                                *ppg = 'F';
                                *(ppg+1)=0;
                            }
                            fl = 0;
                            break;

                case 0x5246 :
                case 0x4652 :
                            if((unsigned)(ppd+1) < (unsigned)((char *)T+strlenT-1)) {           
                                 *(ppg+1) = 0;
                                 *ppg = (unsigned)'L';
                                 ppg=ppg+1;           
                                 strcat((char*)ppg, (char*)ppd+1);
                                 
                            }else {
                                *ppg = 'L';
                                *(ppg+1)=0;
                            }
                            fl = 0;
                            break;
                            
                    
                default :
                    break;
            }
        }

        fl = 1;
    }
}
/* ************************************************************************** */
int main (void) {
    unsigned char superviseur = 0;
    
    leds = 0x00;
    
    FILE *p = fopen("/local/tt.txt","w");
    if (p!=0) leds = 0x01;
    wait(1.);
    m3pi.sensor_auto_calibrate();
    wait(1.);
        
    tk10ms.attach(&isr10ms, 0.01);
    
    while(1) {
        if (flag10ms) {
            flag10ms = 0;
            statcapt = lecture_captsol(tabsensor);
            LFRvect = (LFRvect<<4) + LFRstat;
            
            switch (superviseur) {
                case 0 :
                    leds = 0x00;
                    if (explorer()) {
                        m3pi.cls();
                        sprintf(mess, "simpl");
                        m3pi.print(mess, strlen(mess));
                        leds = 0x03;
                        simplifier(texttab);
                        m3pi.cls();
                        sprintf(mess, "s ok");
                        m3pi.print(mess, strlen(mess));
                        lvwait=0;
                        superviseur = 1;
                    }
                    break;
                    
                case 1 : 
                    leds = 0x01;
                    lvwait++;
                    if (lvwait >= 1500) {
                        leds = 0x06;
                        superviseur = 2;
                    }
                    break;
                
                case 2 :
                    leds = 0x02;
                    if  (rejouer()) {
                        leds = 0x0C;
                        superviseur = 3;
                    }
                    break;
                
                case 3 :
                    leds = 0x03;
                    break;
                    
                default :
                    break;    
            }
        }
    }   
}