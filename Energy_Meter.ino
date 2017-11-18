#include <Arduino.h>
#include <lib/ArduinoJson-v5.11.2.h>

//todo Project description

// System specific constants
#define V1CAL 246 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V2CAL 246 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V3CAL 246 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define I1CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I2CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I3CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I1LAG 0 // Calibration value for how much I1 leads V1, Lead is positive
#define I2LAG 0 // Calibration value for how much I2 leads V2
#define I3LAG 0 // Calibration value for how much I3 leads V3
#define DEVICEID 001 // This needs to be unique to this device
#define IMPORTEXPORT 1 //  Change this to -1 to to change the import export direction


// General constants
#define SUPPLYVOLTS 3.3 // ATMega 328P Supply voltage
#define FILTERSHIFT 13 // For an alpha of 0.000122
#define ADCOFFSET 512 // Initial ADC Offset value
#define NUMSAMPLES 40 // Number of times to sample per cycle -- make sure this is an even number
#define SUPPLYFREQUENCY 50 // Frequency of the supply in Hz
#define SUPPLYMINV 203 // Minimum RMS Volts that will be supplied
#define SUPPLYMAXV 257 // Maximum RMS Volts that will be supplied
#define PLLTIMERRANGE 800 // The max deviation from the average Timer1 value ~ +/- 5 Hz
#define AVRCLOCKSPEED 16000000 // Clock speed of the ATmega328P in Hz
#define PLLLOCKCOUNT 200 // Number of samples for PLL to be considered locked ~ 4 seconds. N.B--Less than 255
#define PLLLOCKRANGE 80 // ADC deviation from offset to enter locked state ~ 1/2 of the time between samples
#define LOOPCYCLES 250 // Cycles to complete before sending data
#define PIDKP 3 // PID KP
#define PIDKI 1 // PID KI
#define PIDKD 0 // PID KD


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
#define PIDK1 (PIDKP + PIDKI + PIDKD) // PID K1
#define PIDK2 (-1*PIDKP - 2*PIDKD) // PID K2
#define PIDK3 (PIDKD) // PID K3


// Pin names and functions
#define V1PIN 3
#define I1PIN 0
#define V2PIN 2
#define I2PIN 6
#define V3PIN 4
#define I3PIN 7
#define RELAY1PIN 5
#define RELAY2PIN 6
#define RELAY3PIN 7
#define PLLLOCKEDPIN 8


// Global Variables - seperated into groups
// Ungrouped variables
unsigned int TimerCount = PLLTIMERAVG; // Timer1 value
byte SampleNum=0; // Sample counter in current cycle
int PllUnlocked=PLLLOCKCOUNT; // Counter reaching zero when PLL is locked
boolean NewCycle=false; // Flag for indicating a new cycle has started
int CycleCount=0; // Counter for number of cycles since last VIPF calculation
long SumTimerCount=0; // Accumulator for total timer cycles elapsed since last VIPF calculation

// Used to phase shift the Voltage to match the current reading
int I1PhaseShift=I1LAG;
int I2PhaseShift=I2LAG;
int I3PhaseShift=I3LAG;

// ADC centre values
int V1Offset=ADCOFFSET;
int V2Offset=ADCOFFSET;
int V3Offset=ADCOFFSET;
int I1Offset=ADCOFFSET;
int I2Offset=ADCOFFSET;
int I3Offset=ADCOFFSET;

// Summed values for current cycle
long SumV1Squared=0;
long SumV2Squared=0;
long SumV3Squared=0;
long SumI1Squared=0;
long SumI2Squared=0;
long SumI3Squared=0;
long SumP1=0;
long SumP2=0;
long SumP3=0;

// Total values for the whole completed cycle
long CycleV1Squared=0;
long CycleV2Squared=0;
long CycleV3Squared=0;
long CycleI1Squared=0;
long CycleI2Squared=0;
long CycleI3Squared=0;
long CycleP1=0;
long CycleP2=0;
long CycleP3=0;

// Summed values for multiple cycles
long TotalV1Squared=0;
long TotalV2Squared=0;
long TotalV3Squared=0;
long TotalI1Squared=0;
long TotalI2Squared=0;
long TotalI3Squared=0;
long TotalP1Import=0;
long TotalP2Import=0;
long TotalP3Import=0;
long TotalP1Export=0;
long TotalP2Export=0;
long TotalP3Export=0;

// Calculated values for the previously completed set of cyles
float V1rms=0;
float V2rms=0;
float V3rms=0;
float I1rms=0;
float I2rms=0;
float I3rms=0;
float RealPower1Import=0;
float RealPower2Import=0;
float RealPower3Import=0;
float RealPower1Export=0;
float RealPower2Export=0;
float RealPower3Export=0;
float ApparentPower1=0;
float ApparentPower2=0;
float ApparentPower3=0;
float PowerFactor1=0;
float PowerFactor2=0;
float PowerFactor3=0;
float Frequency=0;

// Available Units in mWh
long Units1=0;
long Units2=0;
long Units3=0;
long UnitsUsed1=0;
long UnitsUsed2=0;
long UnitsUsed3=0;

// State of relays
byte Relay1State=0;
byte Relay2State=0;
byte Relay3State=0;

// Testvalue
float TV1=0;
float TV2=0;
float TV3=0;

long V[NUMSAMPLES];
int I[NUMSAMPLES];
int E[50];


/*  This will synchronize the timer to the V1 frequency and perform all needed calculations at the end of each cycle
 */


void pllcalcs (int NewV1){

    // Variables that persist between loops
    static int e0=0, e1=0,e2=0;

    // Update the timer value and PLL locked counter at the start of every cycle
    if (SampleNum == 0){

        // PID Controller for the phase locked loop
        // Update the Variables
        e2 = e1;
        e1 = e0;
        e0 = 0 - NewV1;

        // Calculate the new timer value
        TimerCount += (e0 * PIDK1 + e1 * PIDK2 + e2 * PIDK3);
        TimerCount = constrain(TimerCount, PLLTIMERMIN, PLLTIMERMAX);

        // Check if PLL is in lock range and decrement the counter if it is, otherwise set counter to max
        if (abs(NewV1)>PLLLOCKRANGE){
            PllUnlocked = PLLLOCKCOUNT;
        } else if (PllUnlocked) {

            PllUnlocked --;
        }
        // Update the timer
        OCR1A=TimerCount;


    // Last sample of the cycle, perform all calculations and update the variables for the main loop.
    } else if (SampleNum == (NUMSAMPLES-1)) {

        // Update the Cycle Variables
        CycleV1Squared = SumV1Squared;
        CycleV2Squared = SumV2Squared;
        CycleV3Squared = SumV3Squared;
        CycleI1Squared = SumI1Squared;
        CycleI2Squared = SumI2Squared;
        CycleI3Squared = SumI3Squared;
        CycleP1 = (SumP1 * IMPORTEXPORT);
        CycleP2 = (SumP2 * IMPORTEXPORT);
        CycleP3 = (SumP3 * IMPORTEXPORT);
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
    if (CycleP1>=0){
        TotalP1Import += CycleP1;
    } else {
        TotalP1Export -= CycleP1;
    }
    if (CycleP2>=0){
        TotalP2Import += CycleP2;
    } else {
        TotalP2Export -= CycleP2;
    }
    if (CycleP3>=0){
        TotalP3Import += CycleP3;
    } else {
        TotalP3Export -= CycleP3;
    }
    SumTimerCount += (OCR1A+1);

    CycleCount++;
    NewCycle = false;
}


void calculateVIPF(){
    //todo VOltage range

    float TotalTime=0;

    V1rms = V1RATIO * sqrt(((float)TotalV1Squared) / LOOPSAMPLES);
    V2rms = V2RATIO * sqrt(((float)TotalV2Squared) / LOOPSAMPLES);
    V3rms = V3RATIO * sqrt(((float)TotalV3Squared) / LOOPSAMPLES);
    I1rms = I1RATIO * sqrt(((float)TotalI1Squared) / LOOPSAMPLES);
    I2rms = I2RATIO * sqrt(((float)TotalI2Squared) / LOOPSAMPLES);
    I3rms = I3RATIO * sqrt(((float)TotalI3Squared) / LOOPSAMPLES);

    RealPower1Import = (V1RATIO * I1RATIO * (float)TotalP1Import) / LOOPSAMPLES;
    RealPower2Import = (V2RATIO * I2RATIO * (float)TotalP2Import) / LOOPSAMPLES;
    RealPower3Import = (V3RATIO * I3RATIO * (float)TotalP3Import) / LOOPSAMPLES;
    RealPower1Export = (V1RATIO * I1RATIO * (float)TotalP1Export) / LOOPSAMPLES;
    RealPower2Export = (V2RATIO * I2RATIO * (float)TotalP2Export) / LOOPSAMPLES;
    RealPower3Export = (V3RATIO * I3RATIO * (float)TotalP3Export) / LOOPSAMPLES;
    ApparentPower1 = V1rms * I1rms;
    ApparentPower2 = V2rms * I2rms;
    ApparentPower3 = V3rms * I3rms;
    PowerFactor1 = (RealPower1Import + RealPower1Export) / ApparentPower1;
    PowerFactor2 = (RealPower2Import + RealPower2Export) / ApparentPower2;
    PowerFactor3 = (RealPower3Import + RealPower3Export) / ApparentPower3;

    TV1 = RealPower1Import;
    TV2 = RealPower1Export;
    TV3 = ApparentPower1;

    TotalTime = ((float)SumTimerCount * NUMSAMPLES) / AVRCLOCKSPEED; // Time in seconds
    Frequency = (float)CycleCount / TotalTime;

    // Calcualte the units used, 0.5 added for correct rounding
    UnitsUsed1 = long((RealPower1Import * TotalTime / 3.6) + 0.5);
    UnitsUsed2 = long((RealPower2Import * TotalTime / 3.6) + 0.5);
    UnitsUsed3 = long((RealPower3Import * TotalTime / 3.6) + 0.5);

    // Update The unit counter
    Units1 -= UnitsUsed1;
    Units2 -= UnitsUsed2;
    Units3 -= UnitsUsed3;

    // Clear the counters
    TotalV1Squared = 0;
    TotalV2Squared = 0;
    TotalV3Squared = 0;
    TotalI1Squared = 0;
    TotalI2Squared = 0;
    TotalI3Squared = 0;
    TotalP1Import = 0;
    TotalP2Import = 0;
    TotalP3Import = 0;
    TotalP1Export = 0;
    TotalP2Export = 0;
    TotalP3Export = 0;
    SumTimerCount = 0;
    CycleCount = 0;

}


void switchrelays (){

    // Set relay states
    digitalWrite(RELAY1PIN,Relay1State);
    digitalWrite(RELAY2PIN,Relay2State);
    digitalWrite(RELAY3PIN,Relay3State);
    if (PllUnlocked==0) {
        digitalWrite(PLLLOCKEDPIN, HIGH);
    } else {
        digitalWrite(PLLLOCKEDPIN,LOW);
    }

}

void sendresults(){
    // Radio communication
    // todo Radio communication to raspberry PI

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& JsonOutput = jsonBuffer.createObject();

    JsonOutput["ID"] = DEVICEID;
    JsonOutput["Frequency"] = Frequency;
    JsonOutput["PLL"] = PllUnlocked;
    JsonObject& Channel1 = JsonOutput.createNestedObject("Channel1");
    Channel1["VRms"] = V1rms;
    Channel1["IRms"] = I1rms;
    Channel1["PowerFactor"] = PowerFactor1;
    Channel1["PImport"] = RealPower1Import;
    Channel1["PExport"] = RealPower1Export;
    Channel1["UnitsUsed"] = UnitsUsed1;
    Channel1["Units"] = Units1;



    JsonOutput.printTo(Serial);
    Serial.println();



    /*
    Serial.print("V1rms: ");
    Serial.println(V1rms);
    Serial.print("I1rms: ");
    Serial.println(I1rms);
    Serial.print("Power Factor 1: ");
    Serial.println(PowerFactor1);
    Serial.print("Frequency: ");
    Serial.println(Frequency);
    Serial.print("PLL: ");
    Serial.println(PllUnlocked);


    Serial.print("Testvalue1: ");
    Serial.println(TV1);
    Serial.print("Testvalue2: ");
    Serial.println(TV2);
    Serial.print("Testvalue3: ");
    Serial.println(TV3);
    Serial.println(" ");


    Serial.println("V               I");
    for (int i = 0; i < NUMSAMPLES; ++i) {
        Serial.print(V[i]);
        Serial.print("               ");
        Serial.println(I[i]);
    }
    */

}

void setup() {
    // setup pins
    pinMode(RELAY1PIN,OUTPUT);
    digitalWrite(RELAY1PIN,LOW);
    pinMode(RELAY2PIN,OUTPUT);
    digitalWrite(RELAY2PIN,LOW);
    pinMode(RELAY3PIN,OUTPUT);
    digitalWrite(RELAY3PIN,LOW);
    pinMode(PLLLOCKEDPIN,OUTPUT);
    digitalWrite(PLLLOCKEDPIN,LOW);

    // Start Serial and wait for it to initialize
    Serial.begin(9600);
    while (!Serial) {
    }

    // Clear the last 3 bits and change prescaler to 64 = 250kHz
    ADCSRA &= 0xf8;
    ADCSRA |= 0x06;

    noInterrupts();
    TCCR1A = 0; // Clear control registers
    TCCR1B = 0;
    TCNT1  = 0; // Clear counter
    OCR1A = PLLTIMERAVG; // Set compare reg for timer period
    bitSet(TCCR1B,WGM12); // CTC mode
    bitSet(TCCR1B,CS10); // No prescaling
    bitSet(TIMSK1,OCIE1A); // Enable timer 1 compare interrupt
    bitSet(ADCSRA,ADIE); // Enable ADC interrupt
    interrupts();
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
    static int NewV1=0;
    static int NewI1=0;
    static int NewV2=0;
    static int NewI2=0;
    static int NewV3=0;
    static int NewI3=0;
    static int PrevV1=0;
    static int PrevV2=0;
    static int PrevV3=0;
    static int PrevI1=0;
    static int PrevI2=0;
    static int PrevI3=0;
    static long FilterV1Offset=FILTEROFFSET;
    static long FilterV2Offset=FILTEROFFSET;
    static long FilterV3Offset=FILTEROFFSET;
    static long FilterI1Offset=FILTEROFFSET;
    static long FilterI2Offset=FILTEROFFSET;
    static long FilterI3Offset=FILTEROFFSET;
    static int V1Zero=0;
    static int V2Zero=0;
    static int V3Zero=0;
    static int I1Zero=0;
    static int I2Zero=0;
    static int I3Zero=0;
    static byte V1FilterPoint=0;
    static byte V2FilterPoint=0;
    static byte V3FilterPoint=0;
    static byte I1FilterPoint=0;
    static byte I2FilterPoint=0;
    static byte I3FilterPoint=0;

    // Other variables
    int ADCValue=0;
    long PhaseShiftedV=0;
    int TimerNow=0;

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

            /* Store first positive reading for filter calculations and mark where filter should be updated
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
            */
            FilterI1Offset += (NewI1);  // update the filter
            I1Offset=(int)((FilterI1Offset+FILTERROUNDING)>>FILTERSHIFT);

            // Apply phase correction and calculate
            if (I1PhaseShift>=0) { // Current leads voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV1-PrevV1)*(I1PhaseShift)/TimerCount)) + PrevV1;
                SumP1 += (PhaseShiftedV*PrevI1);
                SumV1Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI1Squared += (PrevI1*PrevI1);
                //V[SampleNum] = PhaseShiftedV;
                //I[SampleNum] = PrevI1;
            }
            if (I1PhaseShift<0){ // Current lags voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV1-PrevV1)*(I1PhaseShift)/TimerCount)) + NewV1;
                SumP1 += (PhaseShiftedV*NewI1);
                SumV1Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI1Squared += (NewI1*NewI1);
                //V[SampleNum] = PhaseShiftedV;
                //I[SampleNum] = NewI1;
            }
            V[SampleNum] = NewV1;
            I[SampleNum] = NewI1;

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
            /* Store first positive reading for filter calculations and mark where filter should be updated
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
            */
            FilterI1Offset += (NewI1);  // update the filter
            I1Offset=(int)((FilterI1Offset+FILTERROUNDING)>>FILTERSHIFT);
            // Apply phase correction and calculate
            if (I2PhaseShift>=0) { // Current leads voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV2-PrevV2)*(I2PhaseShift)/TimerCount)) + PrevV2;
                SumP2 += (PhaseShiftedV*PrevI2);
                SumV2Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI2Squared += (PrevI2*PrevI2);
            }
            if (I2PhaseShift<0){ // Current lags voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV2-PrevV2)*(I2PhaseShift)/TimerCount)) + NewV2;
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
            /* Store first positive reading for filter calculations and mark where filter should be updated
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
            */
            FilterI1Offset += (NewI1);  // update the filter
            I1Offset=(int)((FilterI1Offset+FILTERROUNDING)>>FILTERSHIFT);
            // Apply phase correction and calculate
            if (I3PhaseShift>=0) { // Current leads voltage: Interpolate voltage to previous current
                PhaseShiftedV = ((((long)NewV3-PrevV3)*(I3PhaseShift)/TimerCount)) + PrevV3;
                SumP3 += (PhaseShiftedV*PrevI3);
                SumV3Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI3Squared += (PrevI3*PrevI3);
            }
            if (I3PhaseShift<0){ // Current lsgs voltage: Interpolate voltage to new current
                PhaseShiftedV = ((((long)NewV3-PrevV3)*(I3PhaseShift)/TimerCount)) + NewV3;
                SumP3 += (PhaseShiftedV*NewI3);
                SumV3Squared += (PhaseShiftedV*PhaseShiftedV);
                SumI3Squared += (NewI3*NewI3);
            }
            /*
             * Determine CPU Usage
            Serial.print("TimerCount: ");
            Serial.println(TimerCount);
            Serial.print("TimerNow: ");
            Serial.println(TimerNow);
            */
             pllcalcs(NewV1);
            break;
    }

}

