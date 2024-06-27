//   byte recWriteFifoCounter;
// byte recFifoBuffer[256][PACKETLENGTH];  
  
//   //Fifo buffers
//   void checkRecFifo(void);
//   byte recReadFifoCounter;
//   // //Holds command to be sent out
//   void checkSendFifo(void);
//   void incsendWriteFifoCounter(void);
//   byte sendFifoBuffer[256][PACKETLENGTH];  //Way more than needed but better safe than sorry
//   byte sendWriteFifoCounter;               //keeps track of current write position
//   byte sendWriteHighFifoCounter;           //increments every loop to add a HIGH byte
//   byte sendReadFifoCounter;
//   //Repeat fifo
//   int _repeatFIFOTimer = 150;  // set the repeat timer to 0.15 seconds
//   unsigned long _repeatFIFOMillis;
//   unsigned long _repeatFifoMillis[256];
//   byte repeatFifoBuffer[256][PACKETLENGTH];  //Way more than needed but better safe than sorry
//   byte writeRepeatFifoCounter;
//   byte readRepeatFifoCounter;

//   void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
//   memcpy(recFifoBuffer[recWriteFifoCounter], incomingData, PACKETLENGTH);  //copy data into recData array
//   recWriteFifoCounter++;
// }