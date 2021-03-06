 // This file is associated with the file that lists the code for generating a PWM on Timer 1
 // Filename: Timer1.ino
 // Date updated: Spring-2016
 // Version 1.0

// Interrupt related 
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define TOGGLEBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))

//PWM related
unsigned int runDC;
#define DEADTIME    8 // 1 uS
#define PERIOD      250   // 25kHz:320; 20kHz:400 32kHz:250
#define DUTYRATIO   10
unsigned int dutyratio;

//Sine Wave related
boolean runAC;
const uint32_t MAXSPWM = 0xFF - DEADTIME;
const uint32_t MINSPWM = DEADTIME;
const uint32_t AMPSPWM = 128;
const uint32_t LENGTHSPWM = 255;
uint32_t PointerSPWM1;
uint32_t PointerSPWM2;
uint32_t outSPWM1;
uint32_t outSPWM2;
uint32_t pointer_step = 1;
const uint32_t MODULATION_INDEX = 1; 
const bool DIRECTION = 0;
const uint32_t SPWM2_OFFSET = 60;


// State machine and protection related
#define TRUE 1
#define FALSE 0
unsigned char switchstate[5];
int climb;
bool Run_state;
#define INCREASING 1
#define DECREASING -1
#define CLIMB_STEP 1
