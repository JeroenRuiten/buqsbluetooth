#include "Arduino.h"

// Prototypes
void WakeUp();
void GoToSleep();
unsigned long randominterval(unsigned long base, unsigned long div);
int DelayedTick(uint8_t distpid,uint8_t distmod, uint8_t distdiv);
void PagerTick(int times, int interval);
void PagerTickRandom(int times, int interval);
void TickRandom(int times,int interval);
void Tick(int times,int interval);
void ShortTick();
void rTick(int times, int interval);
void PiezoTickRandom(int times, int duration, int frequency, int interval);
void PiezoTick(int times, int duration, int frequency, int interval);
void swarmPagerTick();
void swarmTick();
void rhythmtick();
void vibrate_on();
void vibrate_off();
void timeCorrect();
void timeStamp();
void brighter();
void colorsback();
void aPiezoTick(int times, int duration, int frequency, int interval);
void aTick(int times, int interval);
void play(long duration, int freq);
void ToneUp(long duration, int startfreq, int endfreq);
void ToneDown(long duration, int startfreq, int endfreq);
void messageHandler(struct message received);
void idlechangelow();
void idlechangehigh();
void idlechangenormal();
void everyonehigh();
struct message pollmsg();
byte colorbyte(int red1, int green1, int blue1, int red2, int green2, int blue2);
void RGB2(int red1, int green1, int blue1, int red2, int green2, int blue2);
void initDaytimer(uint8_t choice);
void timerset1();
void timerset2();


// Define variables and constants


// Add setup code
//librarie
#include <avr/wdt.h>
#include <SoftwareSPI.h>
#include <RF22.h>
#include <FiniteStateMachine.h>
//#include <EEPROM.h>
#include <Timer.h>

const int ShiftPWM_latchPin = 8;

//#define SHIFTPWM_USE_TIMER2
//#define SHIFTPWM_NOSPI

const int ShiftPWM_dataPin = 11;
const int ShiftPWM_clockPin = 13;

const bool ShiftPWM_invertOutputs = true;
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>


/// light settings
int bitToSet=0;
int ledres=64;

//pins
static const uint8_t sol = 5;
static const uint8_t piezo = 6;
static const uint8_t pager = 7;
static const uint8_t radioSDN = A3;
static const int latchPin = 8;
static const int clockPin = 13;
static const int dataPin = 11;


//EEPROM addresses
//static const uint8_t sleepbyte = 0;          //[0]
//static const uint8_t distpid_addr = 1;       //[1]
//static const uint8_t distmod_addr = 2;       //[2]
//static const uint8_t distdiv_addr = 3;       //[3]
//static const uint8_t interval_addr = 4;      //[4]
//static const uint8_t intervaldiv_addr = 5;   //[5]
//static const uint8_t id_addr = 6;            //[6]
//static const uint8_t idledutycycle_addr = 7; //[7]

SoftwareSPIClass Software_spi;

//instance of radio
RF22 rf22(10,1);

//global timer
Timer t;

//id

//swarm configuration !todo should be stored in EEPROM
int ritme1[] = {
    128,16,4,0,1, 1,1, 2,1, 3,2, 4,1, 5,0, 6,1, 7,0, 8,1, 9,3, 10,3, 11,0, 12,1, 13,0, 14,1, 15,4};
int ritme2[] = {
    128,16,4,0,3, 1,0, 2,3, 3,0, 4,3, 5,0, 6,3, 7,0, 8,3, 9,0, 10,3, 11,0, 12,3, 13,0, 14,3, 15,0};
int ritme3[] = {
    128,16,4,0,1, 1,3, 2,1, 3,2, 4,1, 5,0, 6,1, 7,0, 8,1, 9,0, 10,1, 11,0, 12,1, 13,0, 14,1, 15,0};
int ritme4[] = {
    176,16,4,1,0, 2,1, 3,0, 5,1, 7,0, 8,0, 9,0, 11,2, 15,0};

//message struct
struct message {
    char header;
    int address;
    int body[20];
};


//globals
//!todo circular message array
int messagebuffer[RF22_MAX_MESSAGE_LEN];
int ticktimes;
//int sleepset = 0;
int highchance = 2;
int hightime = 100;

//config
uint8_t distpid;       //[1]
uint8_t distmod;       //[2]
uint8_t distdiv;       //[3]
uint8_t interval;      //[4]
uint8_t intervaldiv;   //[5]
uint8_t id;            //[6]
uint8_t idledutycycle;  //[7]


//////////////////////////////////////////light section functions


////////////////////////////////////functions init

void hueUpdate(void);
int hueChange(void);
int hueShiftAll(void);
void newHue(int target, int hRange, int tCycle, int cRange, int sat, int val);


//////////////////////////light values

unsigned char maxBrightness = 255;                  //0-255
unsigned char pwmFrequency = 75;                    //
unsigned int numRegisters = 1;                      //buqs uses one register
unsigned int numOutputs = numRegisters*8;
unsigned int numRGBLeds = numRegisters*8/3;

unsigned int fadingMode = 0;                        //start with all LED's off.
unsigned int currentHue = 70;                        //
int minHue = 60;                                    //min of huerange = 0
int maxHue = 240;                                    //0 - 359
int diff = maxHue-minHue;                  //steps between min & maxHue
unsigned long cycleTime = 5000;                      //minimum cycletime
unsigned long cycleTDiff = 4500;                    //random added length to min cycletime
unsigned long cycleTimenow = cycleTime;             //
unsigned long startTime = 0; // start time for the chosen fading mode
unsigned int cycleCount = 2*diff;
int lastHue = 60;
int targetHue = 240;
int curToTarget = targetHue - lastHue;
int hueChanging = 0;
int saturation = 120;
int brightness = 120;
int minSat = saturation;                            //??

bool fixedhue = false;                              //set fixed huerange or floating
long stampTime = 0;
bool lightstate = true;

void lightoff() {
    brightness = 0;
}

void lighton() {
    brightness = 120;
}


/////////////////////////functions should be moved to light.cpp

// hueUpdate updates the color

void hueUpdate(void){

    if(hueChanging == 1)                            //if the hue is changing run hueChange
    {
        currentHue = hueChange();
    }

    else if (hueChanging == 0)                      //otherwise cycle through the hue
    {
        currentHue = hueShiftAll();
    }

}

// cycle from currentHue to the newHue

int hueChange(void){

    unsigned long time = millis()-startTime;


    // OLD CODE
    /*
    long timemodulus = (time % cycleTimenow)/100;        //unclear why its divided by 100. Something to do with milliseconds/seconds?
    long timexcurtotarget = timemodulus*curToTarget;
    long timedivided = (cycleTimenow*0.99)/100;
    long correction = timexcurtotarget / timedivided;
    unsigned long hue = lastHue + correction;
    */

    /* TRY1
    long timemodulus = (time % cycleTimenow)/100;         //where in time are we (0-180 steps)
    long timexcurtotarget = timemodulus*curToTarget;      //where in time * distance (curToTarget)
    long timedivided = (cycleTimenow*0.999)/100;          //what is the stepsize?
    long correctlong = timexcurtotarget / timedivided;     //where in timedistance / time units
    int correction = (int)correctlong;                     // make int from long
    int huemod = lastHue + correction;                  // set hue (modulo for overflow)
    int hue = huemod %359;
    */

    // TRY 2

    int timemodulus = (time % cycleTimenow)/100;         //where in time are we (0-180 steps)
    int timexcurtotarget = timemodulus*curToTarget;      //where in time * distance (curToTarget)
    int timedivided = (cycleTimenow*0.999)/100;          //what is the stepsize?
    int correctlong = timexcurtotarget / timedivided;     //where in timedistance / time units
    int hue = lastHue + correctlong;                  // set hue (modulo for overflow)
    if(hue >= 360)
    {
        hue = hue - 359;
    }
    if(hue < 0)
    {
        hue = hue + 359;
    }

    ShiftPWM.SetAllHSV(hue, saturation, brightness);

    if(hue == targetHue)
    {
        hueChanging = 0;
        startTime = millis();
    }
    return hue;

    // also need to return the saturation and brightness here? - buqs only cycle hue now.
}


// main (default) huechange function:
int hueShiftAll(void){  // Hue shift all LED's

    // update the new cycletime everytime the hue was cycled once
    unsigned int cycleLength = diff + diff;

    if(cycleCount >= cycleLength)
    {
        int chance = random(0,2);
        if (chance >= 1){
            unsigned int deviation = random(10,cycleTDiff);
            cycleTimenow = cycleTime+deviation;
        }
        else{
            cycleTimenow = cycleTime+cycleTDiff;
        }
        cycleCount = 0;
    }

    // main hueshift loop

    unsigned long time = millis()-startTime;
    int correction = diff-((diff*2)*(time % cycleTimenow))/cycleTimenow;
    int biggestHue = maxHue;
    if (minHue>maxHue)
    {
        biggestHue=minHue;
    }


    int hue = maxHue - abs(correction);


    if(hue >= 360)
    {
        hue = hue - 359;
    }
    if(hue < 0)
    {
        hue = hue + 359;
    }

    ShiftPWM.SetAllHSV(hue, saturation, brightness);

    // check if the loop was already completed

    if(lastHue != hue){
        cycleCount = cycleCount + 1;
        lastHue = hue;
    }
    return hue;
}

void brighter()
{
    ShiftPWM.SetAllHSV(90, 80, 255);
}

void colorsback()
{
    brightness = random(120,255);
    saturation = random(80,255);
}

void timeStamp()                        // stamps time for missed time at begin of making sound
{
        stampTime = millis();
}

void timeCorrect()                      // corrects starttime at end of making sound
{

        long correct = millis();
        correct = correct - stampTime;
        startTime = startTime + correct;
}




// handler to change the hues (target hue, huerange etc)
void newHue(int target, int hRange, int tCycle, int cRange, int sat, int val)// updates hue properties (minHue, maxHue, diff)
{
    startTime = millis();
    // with random: unsigned int deviation = random(1,hRange);
    int deviation = hRange;
    minHue = target-(deviation/2);
    maxHue = target+(deviation/2);

    if (minHue <= 0)                // check if hue values are within bounds
    {
        minHue = minHue + 359;
    }

    if (maxHue >= 360)
    {
        maxHue = maxHue - 360;
    }

   // diff = maxHue - minHue;        // this is the old version, causing errors when minHue is bigger than maxHue (minHue 340, maxHue 20 fi)
    diff = deviation;
    cycleTime = tCycle;
    cycleTDiff = cRange;
    targetHue = target;
    curToTarget = targetHue - currentHue;
    if(curToTarget >= 179)
    {
        curToTarget = curToTarget - 360;
    }
    if(curToTarget <= -179)
    {
        curToTarget = curToTarget + 360;
    }
    lastHue = currentHue;
    hueChanging = 1;
    saturation = sat;
    brightness = val;
}


void randomHue()
{
    startTime = millis();
    saturation = random(255);
    brightness = random(100,255);
    targetHue = random(359);


    int deviation = random(1,179);
 /*   if(deviation >= targetHue){                 //check if the deviation goes out of bounds 4 parts
        deviation = deviation / 2;              // part 1
    }
    else if(deviation+targetHue >= 359)
    {
        deviation = deviation / 2;              // part 2
    }
    if(deviation >= targetHue){
        deviation = 0;                          // part 3
    }
    else if(deviation+targetHue >= 359)
    {
        deviation = 0;                          // part 4
    }

  */                                            // not necesary anymore: replaced by below code:


    minHue = targetHue-(deviation/2);
    maxHue = targetHue+(deviation/2);

    if (minHue <= 0)                // check if hue values are within bounds
    {
        minHue = minHue + 359;
    }

    if (maxHue >= 360)
    {
        maxHue = maxHue - 360;
    }


//  diff = maxHue - minHue;           // may cause problems when maxHue is smaller than minHue (20 and 340)
    diff = deviation;
    cycleTime = random(500,3000);
    cycleTDiff = random(cycleTime, 3000);
    hueChanging = 1;
    lastHue = currentHue;
    curToTarget = targetHue - currentHue;
    if(curToTarget >= 179)
    {
        curToTarget = curToTarget - 359;
    }
    if(curToTarget <= -179)
    {
        curToTarget = curToTarget + 359;
    }
}


/*     please do not use anymore.

void randomHue2()
{
    startTime = millis();
    //saturation = random(255);
    //brightness = random(100,255);
    targetHue = random(0,100);
    unsigned int deviation = random(1,100);
    if(deviation >= targetHue){
        deviation = deviation / 2;
    }
    else if(deviation+targetHue >= 100)
    {
        deviation = deviation / 2;
    }
    if(deviation >= targetHue){
        deviation = 0;
    }
    else if(deviation+targetHue >= 100)
    {
        deviation = 0;
    }

    minHue = targetHue-(deviation/2);
    maxHue = targetHue+(deviation/2);
    diff = maxHue - minHue;
    cycleTime = random(500,1000);
    cycleTDiff = random(cycleTime, 3000);
    hueChanging = 1;
    lastHue = currentHue;
    curToTarget = targetHue - currentHue;
    if(curToTarget >= 179)
    {
        curToTarget = curToTarget - 360;
    }
    if(curToTarget <= -179)
    {
        curToTarget = curToTarget + 360;
    }
}


 */


void randomColors(void)
{  // Update random LED to random color. Funky!
    unsigned long updateDelay = 100;
    static unsigned long previousUpdateTime;
    if(millis()-previousUpdateTime > updateDelay){
        previousUpdateTime = millis();
        ShiftPWM.SetHSV(random(numRGBLeds),random(360),255,255);
    }
}


///////////////end of light section


//////////////////////////////////////rhythm section

int tempo(int BPM, int bar)
{
    int beatsinbar = bar/4;
    int onenote = 60000/(BPM*beatsinbar);

    return onenote;
}


void RhythmR(int ritme[])      // [1]BPM,[2]signature,[3]repeat - rhythm [4-n]
{
    int ptime = 100;
    int BPM = tempo(ritme[0], ritme[1]);
    int repeat = ritme[2];
    int beatpos = 3;
    unsigned long currentMS;
    unsigned long lastMS;
    unsigned long compensation_ms = 0;

    for(int counter = 0; counter < repeat; counter++)  // repeat n times
    {
        beatpos = 3;
        compensation_ms = 0;

        for(int beatcount = 0; beatcount < ritme[1]; beatcount++)    // go through rhythm length
        {
            if (ritme[beatpos] == beatcount) {
                lastMS = millis();
                switch (ritme[beatpos+1]) {
                    case 0:
                        break;
                    case 1:
                        rhythmtick();
                        break;
                    case 2:
                        vibrate_on();
                        break;
                    case 3:
                        ToneUp(10, 8000, 3000);
                        break;
                    case 4:
                        rhythmtick();
                        ToneUp(40, 16000, 4000);
                        break;
                    default:
                        break;
                };
                beatpos += 2;
                //RGBres(0,0,20,64,64,64);
                compensation_ms = millis() - lastMS;   // how long did it take to make the sound? Compensate with BPM
            }
            else {
                //RGBres(0,0,0,0,0,0);
                compensation_ms = 0;
            }

            lastMS = millis();

            while((millis() - lastMS) <= (BPM-compensation_ms))
            {

                // back to the main loop?



            }
            vibrate_off();
        }
    }
}


void rhythmtick()
{
    digitalWrite(sol, HIGH);
    delay(20);
    digitalWrite(sol, LOW);
}


void vibrate_on()
{
    digitalWrite(pager, HIGH);
}

void vibrate_off() {
    digitalWrite(pager, LOW);
}

void play(long duration, int freq)
{
    duration *= 1000;
    int period = (1.0 / freq) * 1000000;
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        digitalWrite(piezo,HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(piezo, LOW);
        delayMicroseconds(period / 2);
        elapsed_time += (period);
    }
}

void ToneUp(long duration, int startfreq, int endfreq) {
    duration *= 1000;

    int period = (1.0 / startfreq) * 1000000;
    int endperiod = (1.0 / endfreq) * 1000000;
    int averageperiod = (period+endperiod)/2;
    int times = duration / averageperiod;
    int periodincrease = averageperiod/times;
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        digitalWrite(piezo,HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(piezo, LOW);
        delayMicroseconds(period / 2);
        period += periodincrease;
        elapsed_time += (period);
    }
}

void ToneDown(long duration, int startfreq, int endfreq) {
    duration *= 1000;

    int period = (1.0 / startfreq) * 1000000;
    int endperiod = (1.0 / endfreq) * 1000000;
    int averageperiod = (period+endperiod)/2;
    int times = duration / averageperiod;
    int periodincrease = averageperiod/times;
    long elapsed_time = 0;
    while (elapsed_time < duration) {

        digitalWrite(piezo,HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(piezo, LOW);
        delayMicroseconds(period / 2);
        elapsed_time += (period);
        period += periodincrease;

    }
}
////////////////////////////////////////////////////////////////end of rhythm
void idleTick()
{
    brighter();
    timeStamp();
    Tick(randominterval(5,5),random(10,50));
    timeCorrect();
    colorsback();
}

void idlePagerTick()
{
    brighter();
    timeStamp();
    PagerTick(randominterval(5,5),randominterval(100,90));
    timeCorrect();
    colorsback();
}

void idlePiezo()
{
    brighter();
    timeStamp();
    play(randominterval(200,150),randominterval(5000,4500));
    timeCorrect();
    colorsback();
}

void idlePiezoDown()
{
    brighter();
    timeStamp();
    ToneDown(randominterval(300,250),randominterval(10000,5000),randominterval(500,450));
    timeCorrect();
    colorsback();
}

void idlePiezoTick()
{
    brighter();
    timeStamp();
    PiezoTick(randominterval(4,2),randominterval(200,190), randominterval(9000,7000),randominterval(200,190));
    timeCorrect();
    colorsback();
}

void idlePiezoTickRandom()
{
    brighter();
    timeStamp();
    PiezoTickRandom(randominterval(4,2),randominterval(200,190), randominterval(9000,7000),randominterval(200,190));
    timeCorrect();
    colorsback();
}

void idlePagerTickRandom()
{
    brighter();
    timeStamp();
    PagerTickRandom(randominterval(5,5),randominterval(40,30));
    timeCorrect();
    colorsback();
}

void idleTickRandom()
{
    brighter();
    timeStamp();
    TickRandom(randominterval(5,5),random(10,50));
    timeCorrect();
    colorsback();
}

void idlerhythm()
{
    brighter(); timeStamp();
    //Serial.println("a rhythm!!");
    switch(random(1,5))
    {
        case 1:
            RhythmR(ritme1);
            break;
        case 2:
            RhythmR(ritme2);
            break;
        case 3:
            RhythmR(ritme3);
            break;
        case 4:
            RhythmR(ritme4);
            break;

    }
    timeCorrect(); colorsback();
}//kies uit de lijst met arrays toontjes);

//timer ids

//FSM functions

int timer1 = 99;
int timer2 = 99;
int timer3 = 99;
int timer4 = 99;
int timer5 = 99;
int timer6 = 99;
int timer7 = 99;
int timer8 = 99;
int timer9 = 99;
int timer10 = 99;
float timemod = 1.8;                   //timemod modifies the overall times in behaviors, increase for less activity.

int timer11 = 99;
int timer12 = 99;
int timer13 = 99;
int timer14 = 99;
int timer15 = 99;
int timer16 = 99;
int timer17 = 99;
int timer18 = 99;
int timer19 = 99;
int timer20 = 99;


void idleinit()     //normal activity
{
    //ShiftPWM.SetAllRGB(0, 255, 0);
    //delay(500);
    //timer1 = t.every(randominterval(25000,5000), idlerhythm);
    timer1 = t.every(timemod*randominterval(20000,18000), idleTick);
    timer2 = t.every(timemod*randominterval(30000,28000), idlePagerTick);
    timer3 = t.after((hightime*0.01L)*timemod*random(120000L,180000L), idlechangenormal);
    timer4 = t.every(timemod*randominterval(60000L,30000L), idlePiezo);
    timer5 = t.every(timemod*random(100000L, 200000L), swarmTick);
    timer6 = t.every(timemod*random(100000L, 180000L), swarmPagerTick);

}

void idlehighinit()      //high activity
{
    //ShiftPWM.SetAllRGB(255, 0, 0);
    //delay(500);
    timer1 = t.every(timemod*randominterval(7500,5000), idleTick);
    timer2 = t.every(timemod*randominterval(7500,5000), idlePagerTick);
    timer3 = t.every(timemod*randominterval(7500,5000), idlePiezo);
    timer4 = t.every(timemod*randominterval(7500,5000), idlePiezoDown);
    timer5 = t.after(timemod*randominterval(8000,2000), idlechangehigh);
    timer6 = t.every(timemod*randominterval(7500,5000), idlePiezoTick);
    timer7 = t.every(timemod*randominterval(12000,5000), idlePiezoTickRandom);
    timer8 = t.every(timemod*randominterval(7500,5000), idleTickRandom);
    timer9 = t.every(timemod*randominterval(7500,5000), idlePagerTickRandom);
    //timer10 = t.every(randominterval(10000,9500), randomHue);
}

void idlelowinit()           //low activity
{
    //ShiftPWM.SetAllRGB(0, 0, 255);
    //delay(500);
    timer1 = t.after(timemod*randominterval(20000L,10000L), idlechangelow);
    // timer2 = t.after(randominterval(60000L,55000L), randomHue2);
}

void stopalltimers()
{
    t.update();
    t.stop(timer1);
    t.stop(timer2);
    t.stop(timer3);
    t.stop(timer4);
    t.stop(timer5);
    t.stop(timer6);
    t.stop(timer7);
    t.stop(timer8);
    t.stop(timer9);
    t.stop(timer10);
    ShiftPWM.SetAllHSV(0, 0, 0);
    t.update();
    // newHue(359, 180, 4000, 1, 255, 255);
    //randomHue();
}

void idlefunc()
{
    t.update();
    //  Serial.print("lastHue = ");
    //  Serial.print(lastHue);
    //  Serial.print("/ targetHue = ");
    //  Serial.print(targetHue);
    //  Serial.print("/ curtotarget = ");
    //  Serial.print(curToTarget);
    //  Serial.print("/ hueChanging ");
    //  Serial.println(hueChanging);
    hueUpdate();
    //randomColors();
    messageHandler(pollmsg());


    //Serial.println("idling");
    //receive stuff on the radio
    //if anything comes in pass it to the message handler
    //if it needs to change state, do so
    //some idle ticking,leds
}

void sleepfunc()
{
    t.update();
    //stopalltimers();
    messageHandler(pollmsg());
//    Serial.println("sleeping");
    //receive stuff on the radio
    //if anything comes in pass it to the message handler
}

void swarmfunc()
{
    //do swarm
}


//FSM states
State idle = State(idleinit,idlefunc,stopalltimers);
State idlehigh = State(idlehighinit,idlefunc,stopalltimers);
State idlelow = State(idlelowinit,idlefunc,stopalltimers);
State presentation = State(stopalltimers, idlefunc, stopalltimers);

//FSM
FSM buq = FSM(idle);

//more FSM funcs
void idlechangenormal()
{
    int decision = random(0,10);
    if (decision <= highchance)
    {
        buq.transitionTo(idlehigh);
    }
    if (decision > highchance)
    {
        buq.transitionTo(idlelow);
    }

}

void everyonehigh() //speelkwartier
{
    int decision = random(1,10);
    if (decision <= 5)
    {
        struct message data = {
            'x', 0, {
                0      }
        };
        int len = sizeof(data.body) / sizeof(data.body[0]);
        rf22.send((uint8_t*)&data, sizeof(data));
    }
}


void idlechangehigh()
{
    int decision = random(1,10);
    if (decision <= 2)
    {
        buq.transitionTo(idle);
    }
    if (decision > 2)
    {
        buq.transitionTo(idlelow);
    }
}

void idlechangelow()
{
    int decision = random(1,10);
    if (decision <= highchance)
    {
        buq.transitionTo(idlehigh);
    }
    if (decision > highchance)
    {
        buq.transitionTo(idle);
    }
}


void rhythmfunc()
{
    RhythmR(messagebuffer);
    buq.transitionTo(idle);
}

void tickfunc()
{
    //Serial.println("ticking");
    //Serial.println(ticktimes);
    //Tick(ticktimes);
    DelayedTick(distpid,distmod,distdiv);
    buq.transitionTo(idle);
}

//Final FSM states
State ticking = State(tickfunc,NULL,NULL);
State sleep = State(stopalltimers,sleepfunc,NULL);
State rhythm = State(rhythmfunc,NULL,NULL);
State swarm = State(swarmfunc);

//message handler
void messageHandler(struct message received)
{
    int decision;
    //Serial.println("got msg for:");
    //Serial.print(received.address);
    if (received.address == 0 || received.address == id)
    {

        switch(received.header)
        {
            case 's': //sleep action
                if (received.body[0] == 1)
                {
//                    sleepset = 1;
                    //        Serial.println("sleepset");
                    //        Serial.print(sleepset);
//                    EEPROM.write(sleepbyte, 1);
                    GoToSleep();
                }
                if (received.body[0] == 0)
                {
//                    sleepset = 0;
//                    EEPROM.write(sleepbyte, 0);
                    WakeUp();
                }
                break;

            case 'r': //rhythm action
                if(!buq.isInState(sleep))
                {
                for (int x=0; x < 35; x++)
                {
                    messagebuffer[x] = received.body[x];
                }

                buq.transitionTo(rhythm);
                }
                break;


            case 'x': //speelkwartier
                if(!buq.isInState(sleep))
                {
                buq.transitionTo(idlehigh);
                }
                break;

            case 'm':
                if(!buq.isInState(sleep))
                {
                play(received.body[0]*10, received.body[1]*100);
                }
                break;


            case 'z':
                if(!buq.isInState(sleep))
                {
                if(received.body[2] == 1)
                {
                    delay(random(1,1000));
                    Tick(received.body[0], received.body[1]);
                }
                else if(received.body[2] == 0)
                {
                    Tick(received.body[0], received.body[1]);
                }
                }
                break;

            case 'p':
                if(!buq.isInState(sleep))
                {
                if(received.body[2] == 1)
                {
                    delay(random(1,1000));
                    PagerTick(received.body[0], received.body[1]);
                }
                else if(received.body[2] == 0)
                {
                    PagerTick(received.body[0], received.body[1]);
                }
                }
                break;

            case 'y':
                if(!buq.isInState(sleep))
                {
//                sleepset = 0;
//                EEPROM.write(sleepbyte, 0);
                buq.transitionTo(presentation);
                }
                break;

            case 't': //ticking
                if(!buq.isInState(sleep))
                {
                ticktimes = received.body[0];
                buq.transitionTo(ticking);
                }
                break;

//            case 'c': //configuration
//                distpid = received.body[0];
//                distmod = received.body[1];
//                distdiv = received.body[2];
//                interval = received.body[3];
//                intervaldiv = received.body[4];
//                idledutycycle = received.body[6];
//                if (received.address == id)
//                {
//                    id = received.body [5];
//                }
//                EEPROM.write(distpid_addr, distpid);
//                EEPROM.write(distmod_addr, distmod);
//                EEPROM.write(distdiv_addr, distdiv);
//                EEPROM.write(interval_addr, interval);
//                EEPROM.write(intervaldiv_addr, intervaldiv);
//                EEPROM.write(id_addr, id);
//                EEPROM.write(idledutycycle_addr, idledutycycle);
//                //      Serial.println("distpid: ");
//                //      Serial.println(distpid);
//                //      Serial.println("distmod: ");
//                //      Serial.println(distmod);
//                //      Serial.println("distdiv: ");
//                //      Serial.println(distdiv);
//                //      Serial.println("interval: ");
//                //      Serial.println(interval);
//                //      Serial.println("intervaldiv: ");
//                //      Serial.println(intervaldiv);
//                //      Serial.println("id: ");
//                //      Serial.println(id);
//                break;

            case 'd':
                if(!buq.isInState(sleep))
                {
                initDaytimer(received.body[0]);
                }
                break;

            case 'k':
                if(!buq.isInState(sleep))
                {
                delay(10000);
                }
                break;

            case 'h':
                buq.transitionTo(idlehigh);
                break;

            case 'l':
                buq.transitionTo(idlelow);
                break;

            case 'q':
                hightime = received.body[0];
                highchance = received.body[1];
                buq.transitionTo(idle);
                break;

            case 'a':
                hightime = received.body[0];
                highchance = received.body[1];
                buq.transitionTo(idle);

            case 'v':
                aTick(received.body[0], received.body[1]);
                break;

            case 'b':
                aPiezoTick(received.body[0], received.body[1],received.body[2], received.body[3]);
                break;


        }
    }
}

//radio functions
struct message pollmsg()
{
    {
        rf22.available();
        struct message received ;
        uint8_t buf[RF22_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf22.recv(buf, &len))
        {
            received = (message&)buf;
            message passon = received;
            //
            //      Serial.println(received.header);
            //      Serial.println("id: ");
            //       Serial.println(received.address);
            //      for (int x; x < len; x++ )
            //      {
            //        Serial.print(received.body[x]);
            //      }
            return passon;
        }
        if (!rf22.recv(buf, &len))
        {
            message received = {
                'n', 255, {
                    0         }
            };
            message passon = received;
            return passon;
        }
    }
}

//Tick functions

void swarmTick()
{
    struct message data = {'z', 0, {1,1,1}};
    data.body[0] = random(1,4);
    data.body[1] = random(20,50);
    rTick(data.body[0], data.body[1]);
    int len = sizeof(data.body) / sizeof(data.body[0]);
    rf22.send((uint8_t*)&data, sizeof(data));
    rf22.waitPacketSent(200);
}

void swarmPagerTick()
{
    struct message data = {'p', 0, {1,1,1}};
    data.body[0] = random(1,5);
    data.body[1] = random(30,120);
    PagerTick(data.body[0], data.body[1]);
    int len = sizeof(data.body) / sizeof(data.body[0]);
    rf22.send((uint8_t*)&data, sizeof(data));
    rf22.waitPacketSent(200);
}

void GoToSleep()
{
    buq.transitionTo(sleep);
    lightoff();
}

void WakeUp()
{
    buq.transitionTo(idle);
    lighton();
}


void PiezoTick(int times, int duration, int frequency, int interval)
{
    for (int i=0; i < times; i++)
    {
        play(duration, frequency);
        delay(interval);
    }
}

void PiezoTickRandom(int times, int duration, int frequency, int interval)
{
    for (int i=0; i < times; i++)
    {
        play(randominterval(duration,(duration/3)), randominterval(frequency,(frequency/3)));
        delay(randominterval(interval,(interval/3)));
    }
}

void ShortTick()
{
    digitalWrite(sol, HIGH);
    delay(20);
    digitalWrite(sol, LOW);
}


void aPiezoTick(int times, int duration, int frequency, int interval)
{

    times = randominterval(times, (times-1)/2);
    delay(randominterval(300,280));


    for (int i=0; i < times; i++)
    {

        play(randominterval(interval/2, (interval/3)), frequency);
        delay(randominterval(interval/2, (interval/3)));
    }
}


void aTick(int times,int interval)

{
   // int minterval = interval - 25;

    times = randominterval(times, (times-1)/2);

    //int intervaltick = randominterval(interval, intervaldiv);
    delay(randominterval(300,280));

    for (int i=0; i < times; i++) {
        //Serial.println("TAK................");
        digitalWrite(sol, HIGH);
        delay (30);
        digitalWrite(sol, LOW);
        delay (randominterval(interval/2, (interval/3)));
    }
}


void rTick(int times,int interval)

{
    int minterval = interval - 20;
    times = randominterval(times, (times-1));
    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("TAK................");

        digitalWrite(sol, HIGH);
        delay (30);
        digitalWrite(sol, LOW);
        delay (randominterval(interval,minterval));
    }
}




void Tick(int times,int interval)
{
    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("TAK................");
        digitalWrite(sol, HIGH);
        delay (30);
        digitalWrite(sol, LOW);
        delay (interval);
    }
}

void TickRandom(int times,int interval)
{
    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("TAK................");
        digitalWrite(sol, HIGH);
        delay (randominterval(interval,(interval/3)));
        digitalWrite(sol, LOW);
        delay (randominterval(interval,(interval/3)));
    }
}


void PagerTickRandom(int times, int interval)
{
        if(random(0,2))
        {
            interval = interval/4;
            int low = interval/2;
            int high = interval*2;
            int stretch = high-low;
            int timesdiv = times/stretch;
            int counter = 0;
            int intervaladjust = low;

            //int intervaltick = randominterval(interval, intervaldiv);
            for (int i=0; i < times; i++) {
                //Serial.println("PRRRR................");
                digitalWrite(pager, HIGH);
                delay (intervaladjust);
                digitalWrite(pager, LOW);
                delay (intervaladjust);
                if (counter<timesdiv){
                    counter = counter +1;
                }
                if (counter >=timesdiv)
                {
                    intervaladjust = intervaladjust + 1;
                    counter = 0;
                }

            }
        }
        else
        {

    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("PRRRR................");
        digitalWrite(pager, HIGH);
        delay (randominterval(interval,(interval/3)));
        digitalWrite(pager, LOW);
        delay (randominterval(interval,(interval/3)));
    }
        }
}


void PagerTick(int times, int interval)
{
if(random(0,2))
{
    interval = interval/4;
    int low = interval/2;
    int high = interval*2;
    int stretch = high-low;
    int timesdiv = times/stretch;
    int counter = 0;
    int intervaladjust = low;

    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("PRRRR................");
        digitalWrite(pager, HIGH);
        delay (intervaladjust);
        digitalWrite(pager, LOW);
        delay (intervaladjust);
        if (counter<timesdiv){
            counter = counter +1;
        }
        if (counter >=timesdiv)
        {
            intervaladjust = intervaladjust + 1;
            counter = 0;
        }

    }
}
else
{
    //int intervaltick = randominterval(interval, intervaldiv);
    for (int i=0; i < times; i++) {
        //Serial.println("PRRRR................");
        digitalWrite(pager, HIGH);
        delay (interval);
        digitalWrite(pager, LOW);
        delay (interval);
    }
}
}

int DelayedTick(uint8_t distpid,uint8_t distmod, uint8_t distdiv)
{
    int senddelay = (id * distpid) * distmod;
    //  Serial.println("senddelay: ");
    //  Serial.print(senddelay);
    int randomsenddelay = randominterval(senddelay, distdiv);
    //  Serial.println("randomsenddelay: ");
    //  Serial.print(randomsenddelay);
    int tickevent = t.after(randomsenddelay, swarmTick);
    return 1;
}

//randominterval function
unsigned long randominterval(unsigned long base, unsigned long div)
{
    unsigned long interval = base + random(-div, div);
    return interval;
}

void setup()
{
    //Serial.begin(9600);

    //Serial.println("Serial Comm online");
    //  pinMode(ShiftPWM_latchPin, OUTPUT);
    //pinMode(ShiftPWM_dataPin, OUTPUT);
    //pinMode(ShiftPWM_clockPin, OUTPUT);


    //EEPROM.write(id_addr, 3);
    //watch dog timer resets
    Software_spi.setPins(12, 11, 13);
    wdt_reset();
    wdt_disable();

    //outputs
    pinMode(sol, OUTPUT);
    pinMode(piezo, OUTPUT);
    pinMode(pager, OUTPUT);
    pinMode(radioSDN, OUTPUT);

    pinMode(latchPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    pinMode(clockPin, OUTPUT);

    //random
    randomSeed(analogRead(7));
    ShiftPWM.SetAmountOfRegisters(numRegisters);
    ShiftPWM.SetPinGrouping(1);
    ShiftPWM.Start(pwmFrequency,maxBrightness);
    //turn on the radio
    digitalWrite(radioSDN, LOW);
    ShiftPWM.SetAllRGB(255, 255, 255);
    delay(300);
    ShiftPWM.SetAllRGB(0, 0, 0);

    if (!rf22.init())
    {
        ShiftPWM.SetAllRGB(255, 0, 0);
        delay(500);
        //Serial.println("Radiocontact not possible");  //!todo something to warn us that the radio is not working
    }

    //set saved config
//    distpid = EEPROM.read(distpid_addr);
//    distmod = EEPROM.read(distmod_addr);
//    distdiv = EEPROM.read(distdiv_addr);
//    interval = EEPROM.read(interval_addr);
//    intervaldiv = EEPROM.read(intervaldiv_addr);
//    idledutycycle = EEPROM.read(idledutycycle_addr);
//    id = EEPROM.read(id_addr);
//
//    //read a bit on the EEPROM in case it needs to sleep but has timed-out
//    if (EEPROM.read(sleepbyte) == 1)
//    {
//        buq.transitionTo(sleep);
//    }

    //   Serial.begin(9600);
    //
    //Serial.println("distpid: ");
    //      Serial.println(distpid);
    //      Serial.println("distmod: ");
    //      Serial.println(distmod);
    //      Serial.println("distdiv: ");
    //      Serial.println(distdiv);
    //      Serial.println("interval: ");
    //      Serial.println(interval);
    //      Serial.println("intervaldiv: ");
    //      Serial.println(intervaldiv);
    //      Serial.println("id: ");
    //      Serial.println(id);
    //
    //RGB2(0,0,0,0,0,0);
    //!todo, turn off all the leds on init.

    //only for debugging

    ShortTick();
    delay(300);
    PagerTick(1, 200);
    delay(300);
    PiezoTick(1, 50, 4000, 50);
    delay(300);
    PiezoTick(1, 100, 5000, 50);
    delay(300);
    ShiftPWM.SetAllRGB(255, 255, 255);
    delay(700);
    ShiftPWM.SetAllRGB(0, 0, 0);
    WakeUp();
}

void loop()
{
    buq.update();
    t.update();
}

// REGISTER WRITE writes the register of the shiftregister
void registerWrite(int whichPin, int whichState, byte bitsToSend) {

    // turn off the output so the pins don't light up
    // while you're shifting bits
    digitalWrite(latchPin, LOW);
    // turn on the next highest bit in bitsToSend:
    bitWrite(bitsToSend, whichPin, whichState);

    // shift the bits out:
    shiftOut(dataPin, clockPin, LSBFIRST, bitsToSend);

    // turn on the output so the LEDs can light up:
    digitalWrite(latchPin, HIGH);
}

void RGB2(int red1, int green1, int blue1, int red2, int green2, int blue2)
{
    byte color = colorbyte(red1, green1, blue1, red2, green2, blue2);
    registerWrite(bitToSet, HIGH, color);
}

// COLORBYTE turns zero's and ones into the correct byte
byte colorbyte(int red1, int green1, int blue1, int red2, int green2, int blue2)
{
    byte bitcolor = B11111111;      // make a byte
    int colors[] = {red1, green1, blue1, red2, green2, blue2};   // two arrays corresponding with the led layout
    int bits[] = {3,2,4,6,5,7};                                 // X-X-G6-R6-B6-G5-R5-B5

    for (int steps = 0; steps < 6; steps++) // there are just six RGB's to set
    {
        if(colors[steps]==1)
        {
            bitcolor |= (1 << bits[steps]); //forces nth bit of x to be 1
        }
        else
        {
            bitcolor &= ~(1 << bits[steps]); //forces ntt bit of x to be 0
        }
    }
    return bitcolor;
}

void initDaytimer(uint8_t choice)
{
    buq.transitionTo(sleep);
    t.stop(timer11);
    t.stop(timer12);
    t.stop(timer13);
    t.stop(timer14);
    t.stop(timer15);
    t.stop(timer16);
    t.stop(timer17);
    t.stop(timer18);
    t.stop(timer19);
    t.stop(timer20);
    PagerTick(10,50);
    Tick(10,50);
    PiezoTick(10, 50, 500, 50);
    switch (choice) {
        case 0:
            t.update();
            timerset1();
            t.update();
            break;
        case 1:
            t.update();
            timerset2();
            t.update();
            break;
    }
}



void timerset1()
{
    t.update();
    timer11 = t.after((1000L*60L*60L*4L), WakeUp);
    timer12 = t.after((1000L*60L*60L*(4L+5L)), GoToSleep);
    timer13 = t.after((1000L*60L*60L*(4L+5L+15L)), WakeUp);
    timer14 = t.after((1000L*60L*60L*(4L+5L+15L+9L)), GoToSleep);
    timer15 = t.after((1000L*60L*60L*(4L+5L+15L+9L+15L)), WakeUp);
    timer16 = t.after((1000L*60L*60L*(4L+5L+15L+9L+15L+9L)), GoToSleep);
    timer17 = t.after((1000L*60L*60L*(4L+5L+15L+9L+15L+9L+10L)), WakeUp);
    t.update();
    buq.transitionTo(sleep);
}

void timerset2()
{
    t.update();
    timer11 = t.after((1000L*2L), GoToSleep);
    timer12 = t.after((1000L*4L), WakeUp);
    timer13 = t.after((1000L*6L), GoToSleep);
    timer14 = t.after((1000L*8L), WakeUp);
    timer15 = t.after((1000L*10L), GoToSleep);
    timer16 = t.after((1000L*12L), WakeUp);
    timer17 = t.after((1000L*14L), GoToSleep);
    timer18 = t.after((1000L*16L), WakeUp);
    t.update();
}
