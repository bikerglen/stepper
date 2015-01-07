//---------------------------------------------------------------------------------------------
// includes
//

#include "mbed.h"


//---------------------------------------------------------------------------------------------
// defines
//

// command buffer / interpreter max command line length
#define CMD_MAXLEN 72

// motor driver sample rate
#define SAMPLE_RATE 50000.0

// fraction of acceleration segment that jerk is (+)
#define JERK_POS_FRAC (1.0/3.0)


//---------------------------------------------------------------------------------------------
// enums
//

// motor direction
enum {
    CW = 0,                     // clockwise
    CCW = 1                     // counter clockwise
};


//---------------------------------------------------------------------------------------------
// typedefs
//


//---------------------------------------------------------------------------------------------
// prototypes
//

// command processor prototypes
void InitCommand (void);
bool GetCommand (void);
void ProcessCommand (void);
void CommandToString (void);

// command prototypes
void MoveCommand (void);
void JerkCommand (void);
void SpeedCommand (void);

// movement engine prototypes
void move_Init (void);
void move_DoMove (int32_t j, uint32_t v, int32_t x);
void move_pvajt (int64_t &pt, int64_t &vt, int64_t &at, int64_t p0, int64_t v0, int64_t a0, int64_t j0, int64_t t);
void move_IsrHandler (void);


//---------------------------------------------------------------------------------------------
// globals
//

// use LPC1768 mbed built-in RS-232 / USB converter for console
Serial console(USBTX, USBRX);

// command interpreter
char cmd_buffer[CMD_MAXLEN];
uint8_t cmd_length;
uint8_t cmd_state;
uint8_t word_begin;
uint8_t word_end;
uint8_t word_length;
char cmd_string[CMD_MAXLEN+1];

// global set points (set by speed and jerk commands)
int32_t jerkSetPoint;  // steps per second per second per second
int32_t speedSetPoint; // steps per second

// movement engine state variables
bool gMoveBusy;
int64_t gMoveJerkConst;
int32_t gMoveSample;
int32_t gMovePhase;
int64_t gMoveJerk;
int64_t gMoveAccel;
int64_t gMoveSpeed;
int64_t gMoveFracPos;
int32_t gMovePosition;
int32_t runSamples;
int32_t seg1End, seg2End, seg3End, seg4End, seg5End, seg6End, seg7End;

// LEDs and motors
DigitalOut led1 (LED1);
DigitalOut mStepX (p21);
DigitalOut mDirX (p22);
DigitalOut mStepY (p23);
DigitalOut mDirY (p24);


//---------------------------------------------------------------------------------------------
// main
//

int main (void)
{
    // Hello, world!
    console.printf ("\n\r\n\rHello, world!\n\r");
    
    // turn off LED1
    led1 = 0;
    
    // initialize command interpreter
    InitCommand ();
    
    // pick some defaults for speed and acceleration
    jerkSetPoint = 640000; // steps per second per second per second
    speedSetPoint = 50000; // steps per second
    
    // initialize movement engine
    move_Init ();
    
    // initialize outputs
    led1 = 0;
    mStepX = 0;
    mDirX = 0;
    mStepY = 0;
    mDirY = 0;
    
    // initialize timer ISR at 2x SAMPLE_RATE
    LPC_SC->PCONP |= 1<<1;      // timer0 power on
    LPC_TIM0->MR0 = 239;        // match at 100kHz, count = (pclkf / fs) - 1
    LPC_TIM0->MCR = 3;          // interrupt and reset on match
    LPC_TIM0->TCR = 1;          // enable timer0
    NVIC_SetVector(TIMER0_IRQn,(uint32_t)&move_IsrHandler);
    NVIC_EnableIRQ(TIMER0_IRQn);
        
    // repeat forever
    while (1) {
        
        // run command interpreter
        if (GetCommand ()) {
            // if CR/LF, process command then reset command interpreter
            ProcessCommand ();
            InitCommand ();
        }
        
    }
}


//---------------------------------------------------------------------------------------------
// Console / Command Line Functions
//

void InitCommand (void)
{
    cmd_length = 0;
    cmd_state = 0;
    word_begin = 0;
    word_end = 0;
    word_length = 0;
}


bool GetCommand (void)
{
    uint8_t ch;

    if (cmd_state == 0) {
        cmd_length = 0;
        cmd_buffer[cmd_length] = 0;
        console.printf ("CMD> ");
        cmd_state++;
    } else if (cmd_state == 1) {
        // try to get character
        if (console.readable ()) {
            ch = console.getc ();
            // process character
            if (ch == 0x0d) {                           // return
                // carriage return and linefeed
                console.putc (0x0d);
                console.putc (0x0a);
                cmd_state++;
            } else if ((ch == 0x08) || (ch == 0x7f)) {  // backspace
                if (cmd_length > 0) {
                    console.putc (0x08);
                    console.putc (' ');
                    console.putc (0x08);
                    cmd_buffer[--cmd_length] = 0;
                }
            } else if (ch == 0x15) {                    // ctrl-u is rub out
                while (cmd_length > 0) {
                    console.putc (0x08);
                    console.putc (' ');
                    console.putc (0x08);
                    cmd_buffer[--cmd_length] = 0;
                }
            } else if (ch >= 0x20 && ch <= 0x7e) {      // printable characters
                if (cmd_length < (CMD_MAXLEN - 1)) {
                    console.putc (ch);
                    cmd_buffer[cmd_length++] = ch;
                    cmd_buffer[cmd_length] = 0;
                }
            }
        }
    }
    
    return cmd_state == 2;
}


void ProcessCommand (void)
{
    CommandToString ();
    // console.printf ("command is '%s'\n\r", cmd_string);
    
    if (!strcmp ("mv", cmd_string)) {
        MoveCommand ();
    } else if (!strcmp ("sp", cmd_string)) {
        SpeedCommand ();
    } else if (!strcmp ("jk", cmd_string)) {
        JerkCommand ();
    }
}
    
    
void CommandToString (void)
{
    uint8_t i = 0;
    
    // find beginning of parameter
    word_begin = word_end;
    while ((cmd_buffer[word_begin] == ' ') && (word_begin < cmd_length)) {
        word_begin++;
    }

    // find end of parameter
    word_end = word_begin;
    while ((cmd_buffer[word_end] != ' ') && (word_end < cmd_length)) {
        cmd_string[i++] = cmd_buffer[word_end];
        word_end++;
    }
    
    // terminate string
    cmd_string[i++] = 0;
}
    

//---------------------------------------------------------------------------------------------
// Move / Speed / Accel Commands
//

void MoveCommand (void)
{
    int32_t xSteps;
    
    CommandToString ();
    xSteps = strtol (cmd_string, NULL, 0);
    
    console.printf ("move: x = %d steps\n\r", xSteps);
    
    move_DoMove (jerkSetPoint, speedSetPoint, xSteps);
}


void SpeedCommand (void)
{
    CommandToString ();
    speedSetPoint = strtol (cmd_string, NULL, 0);
    console.printf ("speed: %d steps / second\n\r", speedSetPoint);
}


void JerkCommand (void)
{
    CommandToString ();
    jerkSetPoint = strtol (cmd_string, NULL, 0);
    console.printf ("jerk: %d steps / second / second / second\n\r", jerkSetPoint);
}
    

//---------------------------------------------------------------------------------------------
// Movement Logic
//

void move_Init (void)
{
    gMoveBusy = false;
    gMoveJerkConst = 0;
    gMoveSample = 0;
    gMovePhase = 0;
    gMoveJerk = 0;
    gMoveAccel = 0;
    gMoveSpeed = 0;
    gMoveFracPos = 0;
    gMovePosition = 0;
    runSamples = 0;
    seg1End = 0;
    seg2End = 0;
    seg3End = 0;
    seg4End = 0;
    seg5End = 0;
    seg6End = 0;
    seg7End = 0;
}


void move_DoMove (int32_t j, uint32_t v, int32_t x)
{
    // direction the motor is moving has no bearing on the underlying math
    // remember the requested direction and make the # of steps to move positive 
    int32_t xDir = CW;
    int32_t xSteps = x;
    if (x < 0) {
        xDir = CCW;
        xSteps = -x;
    }

    float jerkFraction = JERK_POS_FRAC;

    console.printf ("j: %d, jf: %f, v: %d, x: %d\n\r", j, jerkFraction, v, x);
    
    // convert move parameters to fixed-point values per sample period
    int64_t jerkAmountInt64  = ((int64_t)j << 40) / ((int64_t)SAMPLE_RATE * SAMPLE_RATE * SAMPLE_RATE);
    int64_t speedTargetInt64 = ((int64_t)v << 40) / ((int64_t)SAMPLE_RATE);
    int64_t moveStepsInt64   = ((int64_t)xSteps << 40);

    // console.printf ("jerkAmountInt64:     %lld\n\r", jerkAmountInt64);
    // console.printf ("speedTargetInt64:    %lld\n\r", speedTargetInt64);
    // console.printf ("moveStepsInt64:      %lld\n\r", moveStepsInt64);

    // calculate the number of samples required to accelerate to the target velocity
    int32_t totalAccelSamples = floor ((double)sqrt ((double)speedTargetInt64 /
                ((double)jerkAmountInt64 * jerkFraction * (1.0 - jerkFraction))));

    // console.printf ("totalAccelSamples:   %d\n\r", totalAccelSamples);

    // based on the fraction of time to spend on (+) jerk during the acceleration
    // phase, calculate the number of samples spent in the each of the three segments 
    // of the accleration and deceleration phases.
    int32_t seg1Samples = floor (jerkFraction * (double)totalAccelSamples);
    int32_t seg3Samples = seg1Samples;
    int32_t seg2Samples = totalAccelSamples - seg1Samples - seg3Samples;
    int32_t seg5Samples = seg3Samples;
    int32_t seg6Samples = seg2Samples;
    int32_t seg7Samples = seg1Samples;

    // calculate the position, velocity, and accleration during the first three segments
    int64_t pf, vf, af;
    int64_t p3, v3, a3;

    move_pvajt (pf, vf, af,  0,  0,  0, +jerkAmountInt64, seg1Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg1)\n\r", pf, vf, af);

    move_pvajt (pf, vf, af, pf, vf, af,                0, seg2Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg2)\n\r", pf, vf, af);

    move_pvajt (p3, v3, a3, pf, vf, af, -jerkAmountInt64, seg3Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg3)\n\r", p3, v3, a3);

    // calculate position after deceleration segments complete 
    // without the constant velocty segment included
    move_pvajt (pf, vf, af, p3, v3, a3, -jerkAmountInt64, seg5Samples);
    move_pvajt (pf, vf, af, pf, vf, af,                0, seg6Samples);
    move_pvajt (pf, vf, af, pf, vf, af, +jerkAmountInt64, seg7Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (excludes seg 4)\n\r", pf, vf, af);

    // calculate number of steps to move during the constant velocity segment
    int64_t seg4Move = moveStepsInt64 - pf;
    int32_t seg4Samples = ceil((double)seg4Move / (double)v3);
    
    console.printf ("Need to move %lld steps in segment 4 in %d samples.\n\r", seg4Move, seg4Samples);

    if (seg4Move < 0) {
        console.printf ("Error: move too short for a complete seven segment profile.\n\r");
        return;
    }

    // console.printf ("samples per segment:  %d %d %d %d %d %d %d\n\r", seg1Samples, seg2Samples,
    //             seg3Samples, seg4Samples, seg5Samples, seg6Samples, seg7Samples);

    // move_pvajt (pf, vf, af, p3, v3, a3,                0, seg4Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg4)\n\r", pf, vf, af);
    // move_pvajt (pf, vf, af, pf, vf, af, -jerkAmountInt64, seg5Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg5)\n\r", pf, vf, af);
    // move_pvajt (pf, vf, af, pf, vf, af,                0, seg6Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg6)\n\r", pf, vf, af);
    // move_pvajt (pf, vf, af, pf, vf, af, +jerkAmountInt64, seg7Samples);
    // console.printf ("pt: %20lld, vt: %20lld, at: %20lld (end of seg7)\n\r", pf, vf, af);

    // precalculate segment boundaries
    seg1End = seg1Samples;
    seg2End = seg1Samples + seg2Samples;
    seg3End = seg1Samples + seg2Samples + seg3Samples;
    seg4End = seg1Samples + seg2Samples + seg3Samples + seg4Samples;
    seg5End = seg1Samples + seg2Samples + seg3Samples + seg4Samples +
        seg5Samples;
    seg6End = seg1Samples + seg2Samples + seg3Samples + seg4Samples +
        seg5Samples + seg6Samples;
    seg7End = seg1Samples + seg2Samples + seg3Samples + seg4Samples +
        seg5Samples + seg6Samples + seg7Samples;
    runSamples = seg7End + 1;

    console.printf ("state machine will run for %d samples\n\r", runSamples);
    console.printf ("boundaries: %d %d %d %d %d %d %d %d\n\r",
        0, seg1End, seg2End, seg3End, seg4End, seg5End, seg6End, seg7End);

    // disable interrupts
    NVIC_DisableIRQ(TIMER0_IRQn);

    // set direction pin
    mDirX = (xDir == CW) ? 0 : 1;
    mDirY = (xDir == CW) ? 0 : 1;

    // start move
    gMoveJerkConst = jerkAmountInt64;
    gMoveSample = 0;
    gMovePhase = 0;
    gMoveJerk = 0;
    gMoveAccel = 0;
    gMoveSpeed = 0;
    gMoveFracPos = 0;
    gMovePosition = 0;
    gMoveBusy = true;

    // re-enable interrups
    NVIC_EnableIRQ(TIMER0_IRQn);
}
    

void move_pvajt (int64_t &pt, int64_t &vt, int64_t &at,
                int64_t p0, int64_t v0, int64_t a0, int64_t j0, int64_t t)
{
    pt = p0 + v0*t + (a0*t*t)/2 + (j0*t*t*t)/6;
    vt = v0 + a0*t + (j0*t*t)/2;
    at = a0 + j0*t;
}


void move_IsrHandler (void)
{
    LPC_TIM0->IR |= 1 << 0;

    if (gMoveBusy) {
        if (gMovePhase == 0) {
            // next phase
            gMovePhase = 1;
            
            // calculate jerk for the current move segment
            if (gMoveSample < seg1End) {
                gMoveJerk = gMoveJerkConst;
            } else if (gMoveSample < seg2End) {
                gMoveJerk = 0;
            } else if (gMoveSample < seg3End) {
                gMoveJerk = -gMoveJerkConst;
            } else if (gMoveSample < seg4End) {
                gMoveJerk = 0;
            } else if (gMoveSample < seg5End) {
                gMoveJerk = -gMoveJerkConst;
            } else if (gMoveSample < seg6End) {
                gMoveJerk = 0;
            } else if (gMoveSample < seg7End) {
                gMoveJerk = +gMoveJerkConst;
            } else {
                gMoveJerk = 0;
            }
            
            // integrate jerk into acceleration
            gMoveAccel = gMoveAccel + gMoveJerk;
    
            // integrate acceleration into velocity
            gMoveSpeed = gMoveSpeed + gMoveAccel;
    
            // integtate velocity into position
            gMoveFracPos = gMoveFracPos + gMoveSpeed;
    
            // if the fractional position wraps around 2^40, increment the position
            if (gMoveFracPos & 0xFFFFFF0000000000L) {
                // take a step!
                mStepX = 1;
                mStepY = 1;
                gMoveFracPos = gMoveFracPos & 0xFFFFFFFFFFL;
                gMovePosition = gMovePosition + 1;
            }
    
            // increment sample number
            gMoveSample++;
        } else {
            // next phase
            gMovePhase = 0;

            // set if time to end move
            if (gMoveSample >= runSamples) {
                gMoveBusy = false;
            }
                            
            // set step pulses low    
            mStepX = 0;
            mStepY = 0;
        }            
    }
            
    // use led1 to indicate when a move is in progress
    led1 = gMoveBusy ? 1 : 0;
}
