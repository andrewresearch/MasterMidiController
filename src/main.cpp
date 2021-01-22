#include <Arduino.h>
#include <USB-MIDI.h>

//debug to turn on/off print to serial
#define DEBUG 1

//define controller analogue pins
#define PITCHBEND 0
#define MODULATION 1

//define controller digital pins
//#define SUSTAIN ?

// Create and bind the MIDI interface
USBMIDI_CREATE_DEFAULT_INSTANCE();

// Digital pins used for keyboard matrix
int lowerSelectPins[5][2] = {{3,2},{5,4},{7,6},{9,8},{11,10}};
int upperSelectPins[6][2] = {{35,34},{37,36},{39,38},{41,40},{43,42},{45,44}};

int lowerKeyPins[8] = {26,27,28,29,30,31,32,33};
int upperKeyPins[8] = {46,47,48,49,50,51,52,53};

// Arrays for scanning
int keyNote[11][2][8] = {};
long keyTime[11][2][8] = {};
int prevPinVal[11][2][8] = {};
int controller[2] = {0};
int stableOff[128] = {0};
float stableOn[128] = {0};

// Globals
bool midiOn = false;
int pitchBendCentre = 0;

// Function declarations

void prepIO(int *selectPin, int *keyPin, int group);
void scanGroup(int *selectPin, int *keyPin, int g);
void stabilityChecker(void);
int controllerCalc(int controller, int readVal);
void scanController(int ctrl);
int velocityOn(long keyInterval);
void sendMidiNoteOn(int note, long keyInterval);
void sendMidiNoteOff(int note, long velocity);
void sendControllerMsg(int control, int ctrlVal);
void sendPitchBendMsg(int pbVal);


 // Debug globals
unsigned long totalMicros = 0;
long cycleCount = 0;

// Debug function declarations
bool oneSecondPassed();
void cycleCheck();
void outputKeyInfo(int g, int k,int s, long t, long i);
void outputTime(long t);
void outputMidiNote(int note, int vel);
void outputControlInfo(int controller, int controlValue);

/** **************************************************************************** **/


/************
 *  SETUP
 */

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  MIDI.begin(MIDI_CHANNEL_OFF); // Don't listen for incoming messages

  pitchBendCentre = analogRead(PITCHBEND); //Check initial value
  
  //prepare IO
  for(int ls = 0; ls < 5; ls++) {
    prepIO(lowerSelectPins[ls],lowerKeyPins,ls);
  }
  for(int us = 0; us < 6; us++) {
    prepIO(upperSelectPins[us],upperKeyPins,(us+5));
  }

  //populate keyNote with midi note numbers
  int midinote = 21; //lowest note
  for (int g=0;g<11;g++) {
    for(int k=0;k<8;k++) {
      for(int s=0;s<2;s++) {  
        keyNote[g][s][k] = midinote;
        if(s==1) {
          midinote += 1;
        }
      }
    }
  }
  
}

/***************
 *  MAIN LOOP
 */

void loop() {
  
  for(int ls = 0; ls < 5; ls++) {
    scanGroup(lowerSelectPins[ls],lowerKeyPins,ls);
  }
  for(int us = 0; us < 6; us++) {
    scanGroup(upperSelectPins[us],upperKeyPins,(us+5));
  }
  for(int c=0; c<2;c++) {
    scanController(c);
  }
  if(DEBUG) cycleCheck();
  //stabilityChecker();
}



/** *************************************************** **/

/**
 *  MAIN FUNCTIONS
 */


// Initialise digital IO
void prepIO(int *selectPin, int *keyPin, int group) {
   for(int s=0;s<2;s++) {
    pinMode(selectPin[s],OUTPUT);
    digitalWrite(selectPin[s],HIGH);
    for(int k=0;k<8;k++) {
      if (s==0) pinMode(keyPin[k],INPUT_PULLUP);
      keyTime[group][s][k] = micros();
      prevPinVal[group][s][k] = 1;
    }
  }
}

// Keyboard scanning
void scanGroup(int *selectPin, int *keyPin, int g) {
  long keyInterval = 0;
  
  for(int s=0;s<2;s++) {
    digitalWrite(selectPin[s],LOW);
    for(int k=0;k<8;k++) {
      int note = keyNote[g][s][k];   
      int pinVal = digitalRead(keyPin[k]);
      if(stableOff[note]>0 && pinVal==1) stableOff[note] +=1; //Counts key off after invalid off
      if(stableOn[note]>0 && pinVal==0) stableOn[note] +=1; //Counts key on after invalid on
      if(pinVal != prevPinVal[g][s][k]) {
        long currKeyTime = micros();
        long prevKeyTime = keyTime[g][s][k];
        long switchInterval = currKeyTime - prevKeyTime; 
        if(switchInterval > 10000) {  // if no switch bounce
          keyTime[g][s][k] = currKeyTime;
          if(DEBUG) outputKeyInfo(g,k,s,currKeyTime,switchInterval);
          
          int keyStatus = (s+1)*(pinVal-prevPinVal[g][s][k]);  // s1 & s2, low-high & high-low
          
          switch(keyStatus) {
            case 1:
              if(DEBUG) Serial.print("S0 OFF = ");
              keyInterval = keyTime[g][0][k] - keyTime[g][1][k];
              if(keyInterval < 3000) {
                //cleanScan = false;
                if(DEBUG) Serial.print(" !!!!! ");
              } else {
                keyTime[g][s][k] = 0;
              }
              if(DEBUG) outputTime(keyInterval);
              if(keyTime[g][1][k]==0 && keyInterval>0) {
                if(DEBUG) Serial.println("VALID OFF");
                sendMidiNoteOff(note,keyInterval);
              } else {
                if(DEBUG) Serial.println("** INVALID OFF **");
                stableOff[note] = 1;
              }
              
            break;
            case 2:
              if(DEBUG) Serial.print("S1 OFF = ");
              keyInterval = keyTime[g][1][k] - keyTime[g][0][k];
              if(keyInterval < 3000) {
                //cleanScan = false;
                if(DEBUG) Serial.print(" !!!!! ");
              } else {
                keyTime[g][s][k] = 0;
              }
              if(DEBUG) outputTime(keyInterval);
              
            break;
            case -1:
              if(DEBUG) Serial.print("S0 ON =  ");
              keyInterval = keyTime[g][0][k] - keyTime[g][1][k];
              if(DEBUG) outputTime(keyInterval);
              
            break;
            case -2:
              if(DEBUG) Serial.print("S1 ON =  ");
              keyInterval = keyTime[g][1][k] - keyTime[g][0][k];
              if(DEBUG) outputTime(keyInterval);
              if(keyTime[g][0][k]!=0 && keyInterval>0) {
                if(DEBUG) Serial.println("VALID ON");
                sendMidiNoteOn(keyNote[g][s][k],keyInterval);
              } else {
                if(DEBUG) Serial.println("** INVALID ON **");
                int midiVel = velocityOn(keyInterval);
                stableOn[note] = 1 + (midiVel/1000); //store velocity as decimal component
              }
              
            break;
            
          } //switch 
          
          
          //if(cleanScan) 
          prevPinVal[g][s][k] = pinVal;
        } //if bounce
        
      } //if pin change
    } //for k
    digitalWrite(selectPin[s],HIGH);
  } //for s

}

// Cleanup rogue readings
void stabilityChecker() {
  for (int n=0; n<128; n++) {
    // If 3 low reads, then send a MIDI OFF
    if(stableOff[n]>7) {
      sendMidiNoteOff(n,0);
      // Reset the stable note value
      stableOff[n]=0;
      if(DEBUG) {
        Serial.print("Cleaned up OFF for: ");
        Serial.println(n);
      }
    }
    if(stableOn[n]>2.0) {
      // Retrieve velocity from decimal component
      int ni = (int)n;
      int midiVel = (int)(1000*(ni-n));
      sendMidiNoteOn(n,midiVel);
      // Reset the stable note value
      stableOn[n]=0;
      if(DEBUG) {
        Serial.print("Cleaned up ON for: ");
        Serial.print(n);
        Serial.print(":");
        Serial.println(midiVel);
      }
    }
  }
}



// Control scanning
void scanController(int ctrl) {
  int cval = analogRead(ctrl);
  if(abs(cval-controller[ctrl]) > 5) {
    int midiVal = controllerCalc(ctrl,cval);
    if (DEBUG) outputControlInfo(ctrl,midiVal);
    if(ctrl==PITCHBEND) {
      sendPitchBendMsg(midiVal);
    } else {
      sendControllerMsg(ctrl,midiVal);
    }
    controller[ctrl] = cval;
  }  
}

// Control value calculation
int controllerCalc(int controller, int readVal) {
  int midiVal = 0;
  switch(controller) {
    case PITCHBEND:
      if(readVal<pitchBendCentre) {
        if(readVal > (pitchBendCentre-100)) readVal = pitchBendCentre;
        midiVal = map(readVal,0,pitchBendCentre, -8000,0);
      } else {
        if(readVal < (pitchBendCentre+100)) readVal = pitchBendCentre;
        midiVal = map(readVal,pitchBendCentre, 1023, 0, 8000);
      }
    break;
    case MODULATION:
      midiVal = readVal/8;
    break;
    default:
      midiVal = readVal/8;
    break;
  }
  return midiVal;
}



// MIDI send functions
int velocityOn(long keyInterval) {
  if(keyInterval < 4000) keyInterval = 10000;
  if(keyInterval < 5000) keyInterval = 5000;
  if(keyInterval > 125000) keyInterval = 125000;
  return ((600000 / keyInterval) + 5);
}

void sendMidiNoteOn(int note, long keyInterval) {
  int midiVel = velocityOn(keyInterval);
  if(DEBUG) outputMidiNote(note,midiVel);
  MIDI.sendNoteOn(note, midiVel, 1);
}

void sendMidiNoteOff(int note, long velocity) {
  //TODO implement noteoff velocity
  if(DEBUG) outputMidiNote(note,0);
  MIDI.sendNoteOff(note,0,1);
}

void sendControllerMsg(int control, int ctrlVal) {
  int controllerNum = 3; //undefined
  switch(control) {
    case MODULATION:
      controllerNum = 1;
    break;
    default:
      controllerNum = 3;
    break;
  }
    MIDI.sendControlChange(controllerNum,ctrlVal,1);
}

void sendPitchBendMsg(int pbVal) {
  if(DEBUG) Serial.println(pbVal);
  MIDI.sendPitchBend(pbVal,1);
}



/********************************************************
 *  DEBUG FUNCTIONS
 */



bool oneSecondPassed() {
  return ((micros() - totalMicros) > 1000000);
}

void cycleCheck() {
  if(oneSecondPassed()) {
    Serial.print(cycleCount);
    Serial.println(" cycles");
    if(midiOn) {
      //MIDI.sendNoteOff(42,0,1);
      midiOn = false;
    } else {
      // Send note 42 with velocity 127 on channel 1
      //MIDI.sendNoteOn(42, 127, 1);
      midiOn = true;
    }
    
    totalMicros = micros();
    cycleCount = 0;
  } else {
    cycleCount += 1;
  }
}

void outputKeyInfo(int g, int k,int s, long t, long i) {
    Serial.print(t);
    Serial.print(" :: ");
    Serial.print(g);
    Serial.print("_");
    Serial.print(k);
    Serial.print("_");
    Serial.print(s);
    Serial.print(" | ");
    Serial.print(i);
    Serial.print(" | ");
}

void outputTime(long t) {
  Serial.println(t);
}

void outputMidiNote(int note, int vel) {
  Serial.println(note);
}

void outputControlInfo(int controller, int controlValue) {
  Serial.print(controller);
  Serial.print(": ");
  Serial.println(controlValue);
}
