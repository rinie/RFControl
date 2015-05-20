#include "RFControl.h"

#ifndef RF_CONTROL_VARDUINO
#include "arduino_functions.h"
#endif
typedef enum RXSTATE {	//
	rxWaiting, rxRecording, rxReady
} RXSTATE;

typedef enum RECORDINGSTATE {	//
	rsOk, rsStart, rsTimeout, rsRestart, rsNextPackage
} RECORDINGSTATE;

char * aRecordingState[] = { "rsOk", "rsStart", "rsTimeout", "rsRestart", "rsNextPackage"};
#define PACKAGE_TIMINGS 4
static unsigned long packageMicros[PACKAGE_TIMINGS];
static unsigned int  packageGapDuration[PACKAGE_TIMINGS];

byte recordingStates[(rsNextPackage+1)*2]; // 5 states space and pulse

#define MAX_PACK 3
#define MIN_PACKAGE_PULSE 32

#define PULSE_LENGTH_DIVIDER 4

#define MIN_FOOTER_LENGTH (3500 / PULSE_LENGTH_DIVIDER)
#define MIN_PULSE_LENGTH (75 / PULSE_LENGTH_DIVIDER)

volatile unsigned int footer_length;
volatile unsigned char state;
volatile int gPackage;
int interruptPin = -1;
bool fIsRf = true;
// RKR modifications
typedef unsigned long ulong; //RKR U N S I G N E D is so verbose
typedef unsigned int uint;
// typedef unsigned char byte;

volatile ulong lastStart;

uint psniStart;
uint psniIndex;

typedef enum PSIX {	//
	psData, psHeader, psFooter
} PSIX;

#define PS_MICRO_ELEMENTS 8
uint psMicroMin[PS_MICRO_ELEMENTS]; // nibble index, 0xF is overflow so max 15
uint psMicroMax[PS_MICRO_ELEMENTS]; // nibble index, 0xF is overflow so max 15
byte psiCount[PS_MICRO_ELEMENTS]; // index frequency
byte psMinMaxCount = 0;

byte psiNibbles[512]; // byteCount + nBytes pulseIndex << 4 | spaceIndex
#define NRELEMENTS(a) (sizeof(a) / sizeof(*(a)))

#ifndef RKR_NO_DURATION
bool new_duration = false;
volatile unsigned int gDuration = 0;
#endif

void handleInterrupt();

unsigned int RFControl::getPulseLengthDivider() {
  return PULSE_LENGTH_DIVIDER;
}

void RFControl::startReceiving(int _interruptPin, bool __fIsRf) {
  footer_length = 0;
  gPackage = 0;
  lastStart = 0;
  state = rxWaiting;
  if(interruptPin != -1) {
    hw_detachInterrupt(interruptPin);
  }
  fIsRf = __fIsRf;
  interruptPin = _interruptPin;
  hw_attachInterrupt(interruptPin, handleInterrupt);
}

void RFControl::stopReceiving() {
  if(interruptPin != -1) {
    hw_detachInterrupt(interruptPin);
  }
  interruptPin = -1;
  state = rxWaiting;
}

bool RFControl::hasData() {
	if ((state == rxRecording) && (lastStart > 0) &&  (psniIndex >= (MIN_PACKAGE_PULSE/2))) {
		  unsigned long currentTime = hw_micros();
			if ((currentTime - lastStart) > (psniIndex >= (MIN_PACKAGE_PULSE)? 750000 : 500000)) {
				state = rxReady;
			}
	}
  if(state == rxReady) {
	  // verify data
	  return true;
  }
  return false;
}
#undef RKR_TEST_SORT
#ifdef RKR_TEST_SORT

static void PrintChar(byte S)
  {
  Serial.write(S);
  }
static void PrintDash(void)
  {
  PrintChar('-');
  }

static void PrintComma(void)
  {
  Serial.print(F(", "));
  }

static void PrintNum(uint x, char c, int digits) {
     // Rinie add space for small digits
     if(c) {
     	PrintChar(c);
 	}
	for (uint i=1, val=10; i < digits; i++, val *= 10) {
		if (x < val) {
			PrintChar(' ');
		}
	}

    Serial.print(x,DEC);
}

static void PrintNumHex(uint x, char c, uint digits) {
	// Rinie add space for small digits
	if(c) {
		PrintChar(c);
	}
	for (uint i=1, val=16; i < digits; i++, val *= 16) {
		if (x < val) {
			PrintChar('0');
		}
	}

	Serial.print(x,HEX);
}
#endif
static void sortMicroMinMax() {
  if ((state != rxReady) || (psiNibbles[0] == 0)){
	  return;
  }
  byte psNewIndex[PS_MICRO_ELEMENTS]; // index frequency
	/*
		[ 1192  384 2120 6024]
		0 1
		1 0
		2 2
		3 3
		 + sort by swap
	 */
	uint prefVal = 0;
	for (byte i = 0; i < psMinMaxCount; i++) {
  	    uint val = -1;
		for (byte j = 0; j < psMinMaxCount; j++) {
			uint curVal = psMicroMin[j];
			if ((curVal < val) && (curVal > prefVal)) {
				val = curVal;
				psNewIndex[i] = j;
			}
		}
		prefVal = val;
	}
#ifdef RKR_TEST_SORT
		  Serial.print(" SORT[");
		  for(unsigned int i=0; i < psMinMaxCount; i++) {
			  unsigned long bucket = ((psMicroMax[i] + psMicroMin[i]) / 2)  * PULSE_LENGTH_DIVIDER;
			  //Serial.print(psiCount[i]);
			  //Serial.write('/');
			  PrintNum(bucket, ' ', 4);
			  //Serial.write(' ');
		  }
		  Serial.println("] ");

		  Serial.print(" INDEX[");
		  for(unsigned int i=0; i < psMinMaxCount; i++) {
			  //Serial.print(psiCount[i]);
			  //Serial.write('/');
			  PrintNum(psNewIndex[i], ' ', 4);
			  //Serial.write(' ');
		  }
		  Serial.println("] ");
#endif
	// swap sort psMicroMin, psMicroMax and psiCount
	bool fDoneSome;
	do {
		fDoneSome = false;
		for (byte j = 0; j < psMinMaxCount-1; j++) {
			if (psMicroMin[j] > psMicroMin[j+1]) {
				uint psMicroMinTemp = psMicroMin[j];
				psMicroMin[j] = psMicroMin[j+1];
				psMicroMin[j+1] = psMicroMinTemp;

				uint psMicroMaxTemp = psMicroMax[j];
				psMicroMax[j] = psMicroMax[j+1];
				psMicroMax[j+1] = psMicroMaxTemp;

				uint psiCountTemp = psiCount[j];
				psiCount[j] = psiCount[j+1];
				psiCount[j+1] = psiCountTemp;

				fDoneSome = true;
			}
		}
	}
	while (fDoneSome);
#ifdef RKR_TEST_SORT
		  Serial.print(F(" AFTERSORT["));
		  for(unsigned int i=0; i < psMinMaxCount; i++) {
			  unsigned long bucket = ((psMicroMax[i] + psMicroMin[i]) / 2)  * PULSE_LENGTH_DIVIDER;
			  //Serial.print(psiCount[i]);
			  //Serial.write('/');
			  PrintNum(bucket, ' ', 4);
			  //Serial.write(' ');
		  }
		  Serial.println("] ");
#endif
	// replace
	uint psniStart1 = 0;
	byte psniSize1 = psiNibbles[psniStart1];
	while (psniSize1 > 0) {
		uint psniStart2 = psniStart1 + psniSize1 + 1;
		byte psniSize2 = psiNibbles[psniStart2];
		for (uint i= 1; i <= psniSize1; i++) {
			byte pulse = ((psiNibbles[i] >> 4) & 0x0F);
			byte space = ((psiNibbles[i]) & 0x0F);
			for (byte j = 0; j < psMinMaxCount; j++) {
				if (pulse == psNewIndex[j]) {
					pulse = j;
					break;
				}
			}
			for (byte j = 0; j < psMinMaxCount; j++) {
				if (space == psNewIndex[j]) {
					space = j;
					break;
				}
			}
#ifdef RKR_TEST_SORT
			if (psiNibbles[i] != ((pulse << 4) | (space))) {
				PrintNumHex(psiNibbles[i], ' ', 2);
				Serial.print(F("->"));
				PrintNumHex(((pulse << 4) | (space)), 0, 2);
			}
#endif
			psiNibbles[i] = (pulse << 4) | (space);
		}
#ifdef RKR_TEST_SORT
		Serial.println();
#endif
		// next
		psniStart1 = psniStart2;
		psniSize1 = psniSize2;
	}
}

void printRecordingStates() {
	    unsigned long now = millis();
	    static unsigned long lastPrint;
	    if (lastPrint == 0) {
			lastPrint = now;
		}
		Serial.print(now-lastPrint);
		lastPrint = now;
		Serial.print(" RecordingStates ");
		for (int i = 0; i < NRELEMENTS(recordingStates); i++) {
			if (recordingStates[i] != 0) {
				bool pulse = (i >= (NRELEMENTS(recordingStates) / 2));
				int j = (pulse)?(i - (NRELEMENTS(recordingStates) / 2)) : i;
				Serial.print(i);
				Serial.print(':');
				if (j < NRELEMENTS(aRecordingState)) {
					Serial.print(aRecordingState[j]);
				}
				else {
					Serial.print(j);
				}
				Serial.print((pulse)? "-Pulse:": "-Space:");
				Serial.print(recordingStates[i]);
				Serial.print(' ');
			}
		}
		Serial.println();
		if (gPackage > 0) {
			Serial.print(" PackageMicros/Gap: ");
			for (int i = 0; i < gPackage && i < NRELEMENTS(packageMicros); i++) {
				Serial.print(packageMicros[i]);
				Serial.print('-');
				Serial.print(packageGapDuration[i] * PULSE_LENGTH_DIVIDER);
				Serial.print(' ');
			}
			Serial.println();
		}
}

unsigned int RFControl::getRawRkr(unsigned char **ppsiNibbles, unsigned int* ppsMinMaxCount, unsigned int** ppsMicroMin, unsigned int** ppsMicroMax, unsigned char** ppsiCount)
{
	*ppsiNibbles = &psiNibbles[0];
	*ppsMinMaxCount = psMinMaxCount;
	*ppsMicroMin = psMicroMin;
	*ppsMicroMax = psMicroMax;
	*ppsiCount = psiCount;
	return gPackage;
}

unsigned int RFControl::getPacket(unsigned char **ppsiNibbles, unsigned int* ppsMinMaxCount, unsigned int** ppsMicroMin, unsigned int** ppsMicroMax, unsigned char** ppsiCount)
{
	sortMicroMinMax();
	printRecordingStates();
	uint psniStart1 = 0;
	byte psniSize1 = psiNibbles[psniStart1];
	*ppsiNibbles = NULL; // if nothing better found?
	while (psniSize1 > 0) {
		uint psniStart2 = psniStart1 + psniSize1 + 1;
		byte psniSize2 = psiNibbles[psniStart2];
		if (psniSize1 == psniSize2) {
			uint i;
			for (i= 1; i <= psniSize1; i++) {
				if (psiNibbles[psniStart1 + i] != psiNibbles[psniStart2 + i]) {
					break;
				}
			}
			if (i >= psniSize1) {
				// all match
				*ppsiNibbles = &psiNibbles[psniStart2];
				break;
			}
		}
		// try next
		psniStart1 = psniStart2;
		psniSize1 = psniSize2;
	}
	*ppsMinMaxCount = psMinMaxCount;
	*ppsMicroMin = psMicroMin;
	*ppsMicroMax = psMicroMax;
	*ppsiCount = psiCount;
	if (*ppsiNibbles != NULL) {
		return gPackage;
	}
	else {
		*ppsiNibbles = &psiNibbles[0];
	}
	return 0;
}

void RFControl::continueReceiving() {
  if(state == rxReady)
  {
    state = rxWaiting;
  }
}

#ifndef RKR_NO_DURATION
unsigned int RFControl::getLastDuration(){
	new_duration = false;
	return gDuration;
}

bool RFControl::existNewDuration(){
	return new_duration;
}
#endif

inline bool probablyFooter(unsigned int duration) {
  return duration >= MIN_FOOTER_LENGTH;
}

#define RKR_FOOTER

bool matchesFooter(unsigned int duration) {
#ifndef RKR_FOOTER
  unsigned int footer_delta = footer_length/4;
  return (footer_length - footer_delta < duration && duration < footer_length + footer_delta);
#else
	unsigned int footer_delta = footer_length/4;
//	return (footer_length - footer_delta < duration && duration < footer_length + footer_delta);
	if ((duration >= MIN_FOOTER_LENGTH) && (footer_length - footer_delta < duration)) {
		footer_length = duration;
		return true;
	}
	return false;
#endif
}

void startRecording(unsigned int duration, unsigned long currentTime, bool fClearRecordingState = false) {
  #ifdef RF_CONTROL_SIMULATE_ARDUINO
  printf(" => start recoding");
  #endif
#ifndef RKR_FOOTER
  footer_length = duration;
#else
  footer_length = MIN_FOOTER_LENGTH;
#endif
	psMinMaxCount = 0;
	psniIndex = 0;
	psniStart = psniIndex;
	if (psniIndex < NRELEMENTS(psiNibbles)) {
		psiNibbles[psniIndex++] = 0;
	}
	if (fClearRecordingState) {
		lastStart = currentTime;
		for (int i = 0; i < NRELEMENTS(recordingStates); i++) {
			recordingStates[i] = 0;
		}
	}
}

/*
 *	psNibbleIndex
 *
 * Lookup/Store timing of pulse and space in psMicroMin/psMicroMax/psiCount array
 * Could use seperate arrays for pulses and spaces but 15 (0xF for overflow) seems enough
 * Store Header as last entry, footer will come last of normal data...
 */
static byte psNibbleIndex(uint pulse, uint space, byte psDataHeaderFooter) {
	byte psNibble = 0;
	uint value = pulse;
	for (int j = 0; j < 2; j++) {
		int i;
		if (value > 0) {
			for (i = 0; i < psMinMaxCount; i++) {
				if (((psMicroMin[i] < 100) || ((psMicroMin[i] - 100) <= value))
					&& (value <= (psMicroMax[i] + 100))) {
					if (psMicroMin[i] > value) { // min
						psMicroMin[i] = value;
					}
					else if (value > psMicroMax[i]) { // max
						psMicroMax[i] = value;
					}
					psiCount[i]++;
					break;
				}
			}
			if (i >= psMinMaxCount && i < 0x0F) { // new value
				if (psDataHeaderFooter == psHeader) {
					i = PS_MICRO_ELEMENTS -1;
				}
				else if (i < PS_MICRO_ELEMENTS - 1) { // except header
					psMinMaxCount++;
				}
				else {
					i = 0x0F; // overflow
				}
				if (i < PS_MICRO_ELEMENTS) {
					psMicroMin[i] = value;
					psMicroMax[i] = value;
					psiCount[i] = 1;
				}
			}
		}
		else {
			i = 0x0F; //invalid data
		}
		psNibble = (psNibble << 4) | (i & 0x0F);
		value = space;
	}
	return psNibble;
}


static int recording(unsigned int duration, uint psCountl) {
	static uint pulseHeader, pulse, spaceHeader, space;
	uint psCount = (psCountl > 0) ? psCountl - 1: psCountl; // check off by one first signal
#ifdef RF_CONTROL_SIMULATE_ARDUINO
  //nice string builder xD
  printf("%s:", sate2string[state]);
  if (data_end[package] < 10)
    printf(" rec_pos=  %i", data_end[package]);
  else if (data_end[package] < 100)
    printf(" rec_pos= %i", data_end[package]);
  else if (data_end[package] < 1000)
    printf(" rec_pos=%i", data_end[package]);
  int pos = data_end[package] - data_start[package];
  if (pos < 10)
    printf(" pack_pos=  %i", pos);
  else if (pos < 100)
    printf(" pack_pos= %i", pos);
  else if (pos < 1000)
    printf(" pack_pos=%i", pos);

  if (duration < 10)
    printf(" duration=    %i", duration);
  else if (duration < 100)
    printf(" duration=   %i", duration);
  else if (duration < 1000)
    printf(" duration=  %i", duration);
  else if (duration < 10000)
    printf(" duration= %i", duration);
  else if (duration < 100000)
    printf(" duration=%i", duration);
#endif
  if ((psCount >= 4) && matchesFooter(duration)) //test for footer (+-25%).
  {
		switch (psCount & 1){ // rest of data
		case 0:
			pulse = duration;
			space = 0;
			if (psniIndex < NRELEMENTS(psiNibbles)) {
				psiNibbles[psniIndex++] = psNibbleIndex(pulse, space, psFooter);
			}
			break;
		case 1:
			space = duration;
			if (psniIndex < NRELEMENTS(psiNibbles)) {
				psiNibbles[psniIndex++] = psNibbleIndex(pulse, space, psFooter);
			}
			break;
		}

	  //recordPackageComplete(duration, package);
	  if (psniIndex - psniStart >= (MIN_PACKAGE_PULSE/2)) {
			if (psniStart < NRELEMENTS(psiNibbles) && (psniIndex - psniStart) <= 0xFF) {
			  psiNibbles[psniStart] = psniIndex - psniStart - 1;
		   	}
		   	psniStart = psniIndex;
			if (psniIndex < NRELEMENTS(psiNibbles)) {
				psiNibbles[psniIndex++] = 0;
			}
		  return rsNextPackage;
	  }
	  else { // restart
	  		psniIndex = psniStart + 1;
		  return rsRestart;
	  }
  }
  else
  {
    //duration isnt a footer? this is the way.
    //if duration higher than the saved footer then the footer isnt a footer -> restart.
    if (duration > footer_length /* + MIN_PULSE_LENGTH */)
    {
      // startRecording(duration);
      return rsTimeout;
    }
    else if (psCount < 4) { // start of signal possibly sync p/s and first data p/s, might miss first p/s timings
		switch (psCount) {
		case 0:
			pulseHeader = duration;
			break;
		case 1:
			spaceHeader = duration;
			break;
		case 2:
			pulse = duration;
			break;
		case 3:
			space = duration;
			byte psNibble = psNibbleIndex(pulse, space, psData);
			if (psniIndex < NRELEMENTS(psiNibbles) - 1) {
				psiNibbles[psniIndex++] = psNibbleIndex(pulseHeader, spaceHeader, psHeader);
				psiNibbles[psniIndex++] = psNibble;
			}
			break;
		}
	}
	else {
		switch (psCount & 1){ // rest of data
		case 0:
			pulse = duration;
			break;
		case 1:
			space = duration;
			if (psniIndex < NRELEMENTS(psiNibbles)) {
				psiNibbles[psniIndex++] = psNibbleIndex(pulse, space, psData);
			}
			break;
		}
	}
  }
  return rsOk;
}

void registerRecordingState(byte rsState, unsigned int duration, unsigned long currentTime, uint psCount, byte pulseOrSpace) {
	int i = rsState + (NRELEMENTS(recordingStates)/2) * pulseOrSpace;
	if (i < NRELEMENTS(recordingStates) && recordingStates[i] < 0xFF) {
		recordingStates[i]++;
	}
	if (rsState == rsNextPackage) {
		if (gPackage < NRELEMENTS(packageMicros)) {
			if (gPackage > 0) {
				packageMicros[gPackage] = currentTime - (packageMicros[gPackage-1] + lastStart);
			}
			else {
				packageMicros[gPackage] = currentTime - lastStart;
			}
			packageGapDuration[gPackage] = duration;
			//lastStart = currentTime;
		}
		//lastStart = currentTime;
	}
}

#define PINHIGH ((PIND & (2 + interruptPin))? 1 : 0)

void handleInterrupt() {
	static bool skip = false;
	static unsigned long lastTime = 0;
	static uint psCount=0;

  //hw_digitalWrite(9, HIGH);
  unsigned long currentTime = hw_micros();
  unsigned int duration = (currentTime - lastTime) / PULSE_LENGTH_DIVIDER;
  //lastTime = currentTime;
  if (skip) {
    skip = false;
    return;
  }
  if (duration >= MIN_PULSE_LENGTH)
  {
#ifndef RKR_NO_DURATION
	new_duration = true;
	gDuration = duration;
#endif
    lastTime = currentTime;
    switch (state)
    {
    case rxWaiting:
      if (probablyFooter(duration) || !fIsRf) {
		state = rxRecording;
        psCount = 0;
		startRecording(duration, currentTime, true);
		registerRecordingState(rsStart, duration, currentTime, psCount, PINHIGH);
	  }
      break;
    case rxRecording: {
      switch (recording(duration, psCount)) {
		case rsTimeout: // timeout in package, restart
			registerRecordingState(rsTimeout, duration, currentTime, psCount, PINHIGH);
	        startRecording(duration, currentTime);
			psCount = 0;
			gPackage = 0;
			// restart wait?
	        state = rxWaiting; // restart wait?
	    break;
	    case rsRestart: // restart package
			registerRecordingState(rsRestart, duration, currentTime, psCount, PINHIGH);
			if (gPackage == 0) {
		        startRecording(duration, currentTime);
		        state = rxWaiting; // restart wait?
			}
			psCount = 0;
			break;
		case rsNextPackage:
			registerRecordingState(rsNextPackage, duration, currentTime, psCount, PINHIGH);
			gPackage++;
			psCount = 0;
	    default: // normal data
	    	psCount++;
	    	break;
	    }
      }
      break;
    }
  }
  else
    skip = true;
  //hw_digitalWrite(9, LOW);
  #ifdef RF_CONTROL_SIMULATE_ARDUINO
  printf("\n");
  #endif
}

// original sending parts

void listenBeforeTalk()
{
  // listen before talk
  unsigned long waited = 0;
  if(interruptPin != -1) {
    while(state == rxRecording) {
      //wait till no rf message is in the air
      waited += 5;
      hw_delayMicroseconds(3); // 5 - some micros for other stuff
      // don't wait longer than 5sec
      if(waited > 5000000) {
        break;
      }
      // some delay between the message in air and the new message send
      // there could be additional repeats following so wait some more time
      if((state != rxRecording) || (gPackage == 0)) {
        waited += 1000000;
        hw_delayMicroseconds(1000000);
      }
    }
    // stop receiving while sending, this method preserves the recording state
    hw_detachInterrupt(interruptPin);
  }
#if 0
  // this prevents loosing the data in the receiving buffer, after sending
  if(data1_ready || data2_ready) {
    state = rxReady;
  }
#else
#endif
  // Serial.print(waited);
  // Serial.print(" ");
}

void afterTalk()
{
  // enable reciving again
  if(interruptPin != -1) {
    hw_attachInterrupt(interruptPin, handleInterrupt);
  }
}


void RFControl::sendByCompressedTimings(int transmitterPin,unsigned long* buckets, char* compressTimings, unsigned int repeats) {
  listenBeforeTalk();
  unsigned int timings_size = strlen(compressTimings);
  hw_pinMode(transmitterPin, OUTPUT);
  for(unsigned int i = 0; i < repeats; i++) {
    hw_digitalWrite(transmitterPin, LOW);
    int state = LOW;
    for(unsigned int j = 0; j < timings_size; j++) {
      state = !state;
      hw_digitalWrite(transmitterPin, state);
      unsigned int index = compressTimings[j] - '0';
      hw_delayMicroseconds(buckets[index]);
    }
  }
  hw_digitalWrite(transmitterPin, LOW);
  afterTalk();
}


void RFControl::sendByTimings(int transmitterPin, unsigned int *timings, unsigned int timings_size, unsigned int repeats) {
  listenBeforeTalk();

  hw_pinMode(transmitterPin, OUTPUT);
  for(unsigned int i = 0; i < repeats; i++) {
    hw_digitalWrite(transmitterPin, LOW);
    int state = LOW;
    for(unsigned int j = 0; j < timings_size; j++) {
      state = !state;
      hw_digitalWrite(transmitterPin, state);
      hw_delayMicroseconds(timings[j]);
    }
  }
  hw_digitalWrite(transmitterPin, LOW);
  afterTalk();
}

