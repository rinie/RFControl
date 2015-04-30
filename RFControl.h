/*
  RFControl.h -
*/
#ifndef ArduinoRf_h
#define ArduinoRf_h
#define RKR_NIBBLEINDEX
#define RKR_NO_DURATION

class RFControl
{
  public:
    static unsigned int getPulseLengthDivider();
    static void startReceiving(int interruptPin);
    static void stopReceiving();
    static bool hasData();
    static void continueReceiving();
#ifndef RKR_NIBBLEINDEX
    static void getRaw(unsigned int **timings, unsigned int* timings_size);
    static bool compressTimings(unsigned int buckets[8], unsigned int *timings, unsigned int timings_size);
    static bool compressTimingsAndSortBuckets(unsigned int buckets[8], unsigned int *timings, unsigned int timings_size);
#else
    static unsigned int getRawRkr(unsigned char **psiNibbles, unsigned int* ppsMinMaxCount, unsigned int** ppsMicroMin, unsigned int** ppsMicroMax, unsigned char** ppsiCount);
#endif
    static void sendByTimings(int transmitterPin, unsigned int *timings, unsigned int timings_size, unsigned int repeats = 3);
    static void sendByCompressedTimings(int transmitterPin, unsigned long* buckets, char* compressTimings, unsigned int repeats = 3);
#ifndef RKR_NO_DURATION
    static unsigned int getLastDuration();
    static bool existNewDuration();
#endif
  private:
    RFControl();
};

#endif
