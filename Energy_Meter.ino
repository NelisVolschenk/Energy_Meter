#include <Arduino.h>

//todo Project description

// System specific constants
#define V1CAL 241 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V2CAL 241 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V3CAL 241 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define I1CAL 91 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I2CAL 91 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I3CAL 91 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I1LAG 400 // Calibration value for how much I1 lags V1
#define I2LAG 400 // Calibration value for how much I2 lags V2
#define I3LAG 400 // Calibration value for how much I3 lags V3


// General constants
#define SUPPLYVOLTS 3.3 // ATMega 328P Supply voltage
#define FILTERSHIFT 13 // For an alpha of 0.000122
#define ADCOFFSET 512 // Initial ADC Offset value
#define NUMSAMPLES 40 // Number of times to sample per cycle -- make sure this is an even number
#define SUPPLYFREQUENCY 50 // Frequency of the supply in Hz
#define PLLTIMERRANGE 900 // The max deviation from the average Timer1 value ~ +/- 5 Hz
#define AVRCLOCKSPEED 16000000 // Clock speed of the ATmega328P in Hz
#define PLLLOCKCOUNT 200 // Number of samples for PLL to be considered locked ~ 4 seconds. N.B--Less than 255
#define PLLLOCKRANGE 80 // ADC deviation from offset to enter locked state ~ 1/2 of the time between samples
#define LOOPCYCLES 250 // Cycles to complete before sending data


// Calculated constants (at compile time)
#define V1RATIO ((V1CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define V2RATIO ((V2CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define V3RATIO ((V3CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define I1RATIO ((I1CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define I2RATIO ((I2CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define I3RATIO ((I3CAL * SUPPLYVOLTS)/1024) // To convert ADC Value to actual value
#define FILTERROUNDING (1<<(FILTERSHIFT-1)) // To add 0.5 and let the system round up.
#define FILTEROFFSET (512L<<FILTERSHIFT) // Starting point of the low pass filter
#define PLLTIMERAVG (AVRCLOCKSPEED/(NUMSAMPLES*SUPPLYFREQUENCY)) // The Timer1 value on which to start the next set of measurements
#define PLLTIMERMIN (PLLTIMERAVG-PLLTIMERRANGE) // Minimum Timer 1 value to start next set of measurements
#define PLLTIMERMAX (PLLTIMERAVG+PLLTIMERRANGE) // Maximum Timer 1 value to start next set of measurements
#define SAMPLEPERIOD (1000000/(SUPPLYFREQUENCY*NUMSAMPLES)) // Sampling period in microseconds
#define LOOPSAMPLES (LOOPCYCLES*NUMSAMPLES) // Number of samples per transmit cycle.


// Pin names and functions

#define V1PIN 0
#define I1PIN 1
#define V2PIN 2
#define I2PIN 3
#define V3PIN 4
#define I3PIN 5


// Global Variables

int V1Offset=ADCOFFSET, V2Offset=ADCOFFSET, V3Offset=ADCOFFSET;
int I1Offset=ADCOFFSET, I2Offset=ADCOFFSET, I3Offset=ADCOFFSET;
long SumV1Squared, SumV2Squared, SumV3Squared, SumI1Squared, SumI2Squared, SumI3Squared;
long CycleV1Squared, CycleV2Squared, CycleV3Squared, CycleI1Squared, CycleI2Squared, CycleI3Squared;
long CycleP1, CycleP2, CycleP3;
long TotalV1Squared, TotalV2Squared, TotalV3Squared, TotalI1Squared, TotalI2Squared, TotalI3Squared;
long TotalP1, TotalP2, TotalP3, TotalP1Export, TotalP2Export, TotalP3Export;
long SumP1, SumP2, SumP3;
int I1PhaseShift, I2PhaseShift, I3PhaseShift;
byte SampleNum;
unsigned int TimerCount = PLLTIMERAVG;
byte PllUnlocked;
boolean NewCycle;
int CycleCount;
long SumTimerCount;
float V1rms,V2rms,V3rms,I1rms,I2rms,I3rms;
float RealPower1, RealPower2, RealPower3, ApparentPower1, ApparentPower2, ApparentPower3;
float PowerFactor1, PowerFactor2, PowerFactor3;
float Frequency;




/*  This will synchronize the timer to the V1 frequency and perform all needed calculations at the end of each cycle
 */


void pllcalcs (int newV1){


    // Variables that persist between loops
    static int oldV1; // The value from the start of the previous cycle
    static int PrevV1; // The previous measurement value
    boolean Rising;

    // Check in which part of the cycle we are
    Rising = (newV1>PrevV1);

    if (SampleNum == 0){ // Start of new cycle

        // If in the rising part of the cycle: This is where we want to be, but adjust the timer if we are moving away
        // from the zero point.
        if (Rising){
            if ( ((newV1<0)&&(newV1<=oldV1)) || ((newV1>0)&&(newV1>=oldV1)) ){
                TimerCount -= newV1;
                TimerCount = constrain(TimerCount,PLLTIMERMIN,PLLTIMERMAX);
            }
            if (abs(newV1)>PLLLOCKRANGE){
                PllUnlocked = PLLLOCKCOUNT; // PLL is now unlocked and needs to relock
            } else if (PllUnlocked) {
                PllUnlocked --; // If PLL is unlocked but in range
            }
            // If in the falling part of the cycle: We don't want to be here, so get out as fast as possible and unlock
        } else {
            TimerCount = PLLTIMERMAX;
            PllUnlocked = PLLLOCKCOUNT;
        }
        // Update the timer and store voltage for use at start of next cycle
        OCR1A=TimerCount;
        oldV1 = newV1;

    // Last sample of the cycle, perform all calculations and update the variables for the main loop.
    } else if (SampleNum == (NUMSAMPLES-1)) {
        // Update the Cycle Variables
        CycleV1Squared = SumV1Squared;
        CycleV2Squared = SumV2Squared;
        CycleV3Squared = SumV3Squared;
        CycleI1Squared = SumI1Squared;
        CycleI2Squared = SumI2Squared;
        CycleI3Squared = SumI3Squared;
        CycleP1 = SumP1;
        CycleP2 = SumP2;
        CycleP3 = SumP3;
        //Clear the old cycle variables
        SumV1Squared = 0;
        SumV2Squared = 0;
        SumV3Squared = 0;
        SumI1Squared = 0;
        SumI2Squared = 0;
        SumI3Squared = 0;
        SumP1 = 0;
        SumP2 = 0;
        SumP3 = 0;
        NewCycle = true;
    }
    // Increment samples and roll over to new cycle if needed
    SampleNum++;
    if (SampleNum >= NUMSAMPLES) SampleNum = 0;

}


void addcycle () {

    TotalV1Squared += CycleV1Squared;
    TotalV2Squared += CycleV2Squared;
    TotalV3Squared += CycleV3Squared;
    TotalI1Squared += CycleI1Squared;
    TotalI2Squared += CycleI2Squared;
    TotalI3Squared += CycleI3Squared;
    TotalP1 += CycleP1;
    TotalP2 += CycleP2;
    TotalP3 += CycleP3;
    SumTimerCount += (OCR1A+1);

    CycleCount++;
    NewCycle = false;
}


void calculateVIPF(){

    V1rms = V1RATIO * sqrt(((float)TotalV1Squared)/LOOPSAMPLES);
    V2rms = V2RATIO * sqrt(((float)TotalV2Squared)/LOOPSAMPLES);
    V3rms = V3RATIO * sqrt(((float)TotalV3Squared)/LOOPSAMPLES);
    I1rms = I1RATIO * sqrt(((float)TotalI1Squared)/LOOPSAMPLES);
    I2rms = I2RATIO * sqrt(((float)TotalI2Squared)/LOOPSAMPLES);
    I3rms = I3RATIO * sqrt(((float)TotalI3Squared)/LOOPSAMPLES);

    RealPower1 = (V1RATIO * I1RATIO * (float)TotalP1)/LOOPSAMPLES;
    RealPower2 = (V2RATIO * I2RATIO * (float)TotalP2)/LOOPSAMPLES;
    RealPower3 = (V3RATIO * I3RATIO * (float)TotalP3)/LOOPSAMPLES;
    ApparentPower1 = V1rms * I1rms;
    ApparentPower2 = V2rms * I2rms;
    ApparentPower3 = V3rms * I2rms;
    PowerFactor1 = RealPower1 / ApparentPower1;
    PowerFactor2 = RealPower2 / ApparentPower2;
    PowerFactor3 = RealPower3 / ApparentPower3;

    Frequency = ((float)CycleCount*16000000)/((float)SumTimerCount*NUMSAMPLES);

    TotalV1Squared = 0;
    TotalV2Squared = 0;
    TotalV3Squared = 0;
    TotalI1Squared = 0;
    TotalI2Squared = 0;
    TotalI3Squared = 0;
    TotalP1 = 0;
    TotalP2 = 0;
    TotalP3 = 0;
    SumTimerCount = 0;
    CycleCount = 0;

}

void switchrelays (){
    // Import/export
    // accumulators
    // communication

}

void sendresults(){

}

void setup() {
    // setup pins

}


void loop() {
    // Calculate averages and transmit data to base station
    if (NewCycle) {
        addcycle();
    }
    if (CycleCount >= LOOPCYCLES){
        calculateVIPF();
        switchrelays();
        sendresults();
    }
}

// Timer interrupt
ISR(TIMER1_COMPA_vect){
    ADMUX = _BV(REFS0) | V1PIN; // Set ADC conversion to start on V1Pin
    ADCSRA |= _BV(ADSC); // Start ADC conversion
}

// ADC interrupt
ISR(ADC_vect){
    // Variables that persist between conversions
    static int NewV1, NewI1, NewV2, NewI2, NewV3, NewI3;
    static int PrevV1, PrevV2, PrevV3, PrevI1, PrevI2, PrevI3;
    static long FilterV1Offset=FILTEROFFSET, FilterV2Offset=FILTEROFFSET, FilterV3Offset=FILTEROFFSET;
    static long FilterI1Offset=FILTEROFFSET, FilterI2Offset=FILTEROFFSET, FilterI3Offset=FILTEROFFSET;
    static int V1Zero, V2Zero, V3Zero, I1Zero, I2Zero, I3Zero;
    static byte V1FilterPoint, V2FilterPoint, V3FilterPoint, I1FilterPoint, I2FilterPoint, I3FilterPoint;
    static int VTime, ITime;
    // Other variables
    int ADCValue;
    long PhaseShiftedV=0;
    int TimerNow;

    // Collect the result from the registers and combine
    TimerNow = TCNT1;
    ADCValue = ADCL;
    ADCValue |= ADCH<<8;

    // Determine which conversion completed and go to next conversion
    switch(ADMUX & 0x0f){

        case V1PIN: // V1 Just completed
            ADMUX = _BV(REFS0) | I1PIN; // Set parameters for ADC conversion to start on I1Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for V1
            PrevV1 = NewV1;
            NewV1 =  ADCValue - V1Offset;
            VTime = TimerNow;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewV1>=0)&&(PrevV1<0)) {
                V1Zero = NewV1;
                V1FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (V1FilterPoint>=NUMSAMPLES) {
                    V1FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == V1FilterPoint) {
                FilterV1Offset += (V1Zero+NewV1)>>1;
                V1Offset=(int)((FilterV1Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            break;

        case I1PIN: // I1 Just completed
            ADMUX = _BV(REFS0) | V2PIN; // Set ADC conversion to start on V2Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for I1
            PrevI1 = NewI1;
            NewI1 =  ADCValue - I1Offset;
            ITime = TimerNow;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewI1>=0)&&(PrevI1<0)) {
                I1Zero = NewI1;
                I1FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (I1FilterPoint>=NUMSAMPLES) {
                    I1FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == I1FilterPoint) {
                FilterI1Offset += (I1Zero+NewI1)>>1;
                I1Offset=(int)((FilterI1Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            // Apply phase correction and calculate
            if (I1PhaseShift>=0) { // Current lags voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV1-PrevV1)*(I1PhaseShift)*SAMPLEPERIOD/TimerCount)>>6) + PrevV1;
                SumP1 += (PhaseShiftedV*PrevI1);
                SumV1Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI1Squared += (PrevI1*PrevI1);
            }
            if (I1PhaseShift<0){ // Current leads voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV1-PrevV1)*(I1PhaseShift+1)*SAMPLEPERIOD/TimerCount)>>6) + PrevV1;
                SumP1 += (PhaseShiftedV*NewI1);
                SumV1Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI1Squared += (NewI1*NewI1);
            }
            break;

        case V2PIN: // V2 Just completed
            ADMUX = _BV(REFS0) | I2PIN; // Set parameters for ADC conversion to start on I1Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for V2
            PrevV2 = NewV2;
            NewV2 =  ADCValue - V2Offset;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewV2>=0)&&(PrevV2<0)) {
                V2Zero = NewV2;
                V2FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (V2FilterPoint>=NUMSAMPLES) {
                    V2FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == V2FilterPoint) {
                FilterV2Offset += (V2Zero+NewV2)>>1;
                V2Offset=(int)((FilterV2Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            break;

        case I2PIN: // I2 Just completed
            ADMUX = _BV(REFS0) | V3PIN; // Set ADC conversion to start on V2Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for I2
            PrevI2 = NewI2;
            NewI2 =  ADCValue - I2Offset;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewI2>=0)&&(PrevI2<0)) {
                I2Zero = NewI2;
                I2FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (I2FilterPoint>=NUMSAMPLES) {
                    I2FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == I2FilterPoint) {
                FilterI2Offset += (I2Zero+NewI2)>>1;
                I2Offset=(int)((FilterI2Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            // Apply phase correction and calculate
            if (I2PhaseShift>=0) { // Current lags voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV2-PrevV2)*(I2PhaseShift)*SAMPLEPERIOD/TimerCount)>>6) + PrevV2;
                SumP2 += (PhaseShiftedV*PrevI2);
                SumV2Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI2Squared += (PrevI2*PrevI2);
            }
            if (I2PhaseShift<0){ // Current leads voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV2-PrevV2)*(I2PhaseShift+1)*SAMPLEPERIOD/TimerCount)>>6) + PrevV2;
                SumP2 += (PhaseShiftedV*NewI2);
                SumV2Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI2Squared += (NewI2*NewI2);
            }
            break;

        case V3PIN: // V3 Just completed
            ADMUX = _BV(REFS0) | I3PIN; // Set parameters for ADC conversion to start on I1Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for V2
            PrevV3 = NewV3;
            NewV3 =  ADCValue - V3Offset;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewV3>=0)&&(PrevV3<0)) {
                V3Zero = NewV3;
                V3FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (V3FilterPoint>=NUMSAMPLES) {
                    V3FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == V3FilterPoint) {
                FilterV3Offset += (V3Zero+NewV3)>>1;
                V3Offset=(int)((FilterV3Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            break;

        case I3PIN: // I3 Just completed
            // No new ADC conversion here as this is the last in the list

            // Update variables for I3
            PrevI3 = NewI3;
            NewI3 =  ADCValue - I3Offset;
            // Store first positive reading for filter calculations and mark where filter should be updated
            if ((NewI3>=0)&&(PrevI3<0)) {
                I3Zero = NewI3;
                I3FilterPoint = SampleNum + (NUMSAMPLES/2);
                if (I3FilterPoint>=NUMSAMPLES) {
                    I3FilterPoint -= NUMSAMPLES;
                }
            }
            // Update low pass filter at centre of wave
            if (SampleNum == I3FilterPoint) {
                FilterI3Offset += (I3Zero+NewI3)>>1;
                I3Offset=(int)((FilterI3Offset+FILTERROUNDING)>>FILTERSHIFT);
            }
            // Apply phase correction and calculate
            if (I3PhaseShift>=0) { // Current lags voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV3-PrevV3)*(I3PhaseShift)*SAMPLEPERIOD/TimerCount)>>6) + PrevV3;
                SumP3 += (PhaseShiftedV*PrevI3);
                SumV3Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI3Squared += (PrevI3*PrevI3);
            }
            if (I3PhaseShift<0){ // Current leads voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV3-PrevV3)*(I3PhaseShift+1)*SAMPLEPERIOD/TimerCount)>>6) + PrevV3;
                SumP3 += (PhaseShiftedV*NewI3);
                SumV3Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI3Squared += (NewI3*NewI3);
            }
            pllcalcs(NewV1);
            break;
    }

}

