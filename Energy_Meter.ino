#include <Arduino.h>
#include <lib/ArduinoJson-v5.11.2.h>

//todo Project description

// System specific constants
#define V1CAL 249 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V2CAL 246 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define V3CAL 246 // Calculated value is 230:10.5 for transformer x 11:1 for resistor divider = 241
#define I1CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I2CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I3CAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91
#define I1LAG 0 // Calibration value for how much I1 leads V1, Lead is positive
#define I2LAG 0 // Calibration value for how much I2 leads V2
#define I3LAG 0 // Calibration value for how much I3 leads V3
#define DEVICEID 001 // This needs to be unique to this device
#define IMPORTEXPORT -1 //  Change this to -1 to to change the import export direction


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
long Units1=1000000000;
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
    static int e0=0;
    static int e1=0;
    static int e2=0;

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


void sendjson(int vadc, int iadc){

    StaticJsonBuffer<100> jsonBuffer;
    JsonObject& JsonOutput = jsonBuffer.createObject();

    JsonOutput["V"] = vadc;
    JsonOutput["I"] = iadc;

    JsonOutput.printTo(Serial);
    Serial.println();

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
    Serial.begin(500000);
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
    static long FilterV1Offset=512L<<13;

    // Temporary for quick serial comm
    static int VAdc=0;
    String newline;

    // Other variables
    int ADCValue=0;

    // Collect the result from the registers and combine
    ADCValue = ADCL;
    ADCValue |= ADCH<<8;

    // Determine which conversion completed and go to next conversion
    switch(ADMUX & 0x0f){

        case V1PIN: // V1 Just completed
            ADMUX = _BV(REFS0) | I1PIN; // Set parameters for ADC conversion to start on I1Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion
            // Update variables for V1
            NewV1 =  ADCValue - V1Offset;

            // Update the Low Pass filter
            FilterV1Offset += (NewV1);
            V1Offset=(int)((FilterV1Offset+FILTERROUNDING)>>FILTERSHIFT);

            VAdc = ADCValue;

            break;


        case I1PIN: // I1 Just completed

            V[SampleNum] = VAdc;
            I[SampleNum] = ADCValue;

            if (SampleNum == NUMSAMPLES - 1){
                noInterrupts();
                newline = "{\"V\":\"New\"}";
                Serial.println(newline);
                Serial.println(OCR1A);
                Serial.println(PllUnlocked);
                for (int i=0; i < NUMSAMPLES; i++){
                    sendjson(V[i],I[i]);
                }

                interrupts();
            }

            pllcalcs(NewV1);
            break;
    }

}

