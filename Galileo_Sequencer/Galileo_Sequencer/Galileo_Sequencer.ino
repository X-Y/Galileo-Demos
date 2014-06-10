/*
 Galileo Sequencer
 This sketch is used for a demo of the Intel Galileo board. When it was launched in Rome Maker fair 2013, 
 we made a sequencer based on Galileo, SparkFun MP3 shield and the sketch ReadMP3fromSD originally created 
 by Nathan Seidle.
 
 The sequencer has 2 knobs, 4 buttons and 8 leds on its interface. When activated, it loops through and
 plays 8 segments of sound samples repeatedly. At the mean time, the LEDs will light up according to which
 segment you¡¯re listening to. You can use the buttons and knobs to customize and add creativity into the sequence.

 If you press the big red ¡°random¡± button, the whole sequence will be randomized. It often creates a good
 sequence out of randomness. The ¡°try¡± button will make it play the currently selected sample, without 
 making any changes to the sequence. You can select different samples by turning the ¡°sample selection¡± knob.
 By pushing the ¡°set¡± button, you change the currently playing segment into the selected sample. And the 
 ¡°pause¡± button will stop the playback as long as you¡¯re holding it. Turn the knob marked ¡°tempo select¡±, you
 limit the sequence length to 1 to 8 segments.
 
 By Xun Yang
 2013

 ================The original comment in ReadMP3fromSD ===============

 4-28-2011
 Spark Fun Electronics 2011
 Nathan Seidle
 
 This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This example code plays a MP3 from the SD card called 'track001.mp3'. The theory is that you can load a 
 microSD card up with a bunch of MP3s and then play a given 'track' depending on some sort of input such 
 as which pin is pulled low.
 
 It relies on the sdfatlib from Bill Greiman: 
 http://code.google.com/p/sdfatlib/
 You will need to download and install his library. To compile, you MUST change Sd2PinMap.h of the SDfatlib! 
 The default SS_PIN = 10;. You must change this line under the ATmega328/Arduino area of code to 
 uint8_t const SS_PIN = 9;. This will cause the sdfatlib to use pin 9 as the 'chip select' for the 
 microSD card on pin 9 of the Arduino so that the layout of the shield works.
 
 Attach the shield to an Arduino. Load code (after editing Sd2PinMap.h) then open the terminal at 57600bps. This 
 example shows that it takes ~30ms to load up the VS1053 buffer. We can then do whatever we want for ~100ms 
 before we need to return to filling the buffer (for another 30ms).
 
 This code is heavily based on the example code I wrote to control the MP3 shield found here:
 http://www.sparkfun.com/products/9736
 This example code extends the previous example by reading the MP3 from an SD card and file rather than from internal
 memory of the ATmega. Because the current MP3 shield does not have a microSD socket, you will need to add the microSD 
 shield to your Arduino stack.
 
 The main gotcha from all of this is that you have to make sure your CS pins for each device on an SPI bus is carefully
 declared. For the SS pin (aka CS) on the SD FAT libaray, you need to correctly set it within Sd2PinMap.h. The default 
 pin in Sd2PinMap.h is 10. If you're using the SparkFun microSD shield with the SparkFun MP3 shield, the SD CS pin 
 is pin 9. 
 
 Four pins are needed to control the VS1503:
 DREQ
 CS
 DCS
 Reset (optional but good to have access to)
 Plus the SPI bus
 
 Only the SPI bus pins and another CS pin are needed to control the microSD card.
 
 What surprised me is the fact that with a normal MP3 we can do other things for up to 100ms while the MP3 IC crunches
 through it's fairly large buffer of 2048 bytes. As long as you keep your sensor checks or serial reporting to under 
 100ms and leave ~30ms to then replenish the MP3 buffer, you can do quite a lot while the MP3 is playing glitch free.
 
 */

#include <SPI.h>
#include <SD.h>

#define CHUNK_SIZE 32
#define PAGE_SIZE 4096

//Create the variables to be used by SdFat Library
File track;

//This is the name of the file on the microSD card you would like to play
//Stick with normal 8.3 nomeclature. All lower-case works well.
//Note: you must name the tracks on the SD card with 001, 002, 003, etc. 
//For example, the code is expecting to play 'track002.mp3', not track2.mp3.
char trackName[] = "track001.mp3";
int trackNumber = 1;

char errorMsg[100]; //This is a generic array used for sprintf of error messages

#define TRUE  0
#define FALSE  1

//MP3 Player Shield pin mapping. See the schematic
#define MP3_XCS 6 //Control Chip Select Pin (for accessing SPI Control/Status registers)
#define MP3_XDCS 7 //Data Chip Select / BSYNC Pin
#define MP3_DREQ 2 //Data Request Pin: Player asks for more data
#define MP3_RESET 8 //Reset is active low
//Remember you have to edit the Sd2PinMap.h of the sdfatlib library to correct control the SD card.

//VS10xx SCI Registers
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0A
#define SCI_VOL 0x0B
#define SCI_AICTRL0 0x0C
#define SCI_AICTRL1 0x0D
#define SCI_AICTRL2 0x0E
#define SCI_AICTRL3 0x0F


//Synth
#define notes_len_max 8
#define num_sounds 49
int pin_test=3;
int pin_save=9;
int pin_silent=10;
int pin_random=5;
int knob_choose_pin=A0;
int knob_nn_pin=A1;//number of notes knob

int sequence[notes_len_max];
int curr_sound_num;

int num_notes=notes_len_max;
float last_note_length=1;

//leds-74H595
#define latchPin A5

void setup() {
  pinMode(MP3_DREQ, INPUT);
  pinMode(MP3_XCS, OUTPUT);
  pinMode(MP3_XDCS, OUTPUT);
  pinMode(MP3_RESET, OUTPUT);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  digitalWrite(MP3_XCS, HIGH); //Deselect Control
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  digitalWrite(MP3_RESET, LOW); //Put VS1053 into hardware reset

  Serial.begin(9600); //Use serial for debugging 

  //Serial.println("Type any character to start");
  //while (Serial.read() <= 0) {}

  Serial.println("MP3 Testing");

  if (!SD.begin(0)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("SD card initialized.");

  //From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz. 
  //Internal clock multiplier is 1.0x after power up. 
  //Therefore, max SPI speed is 1.75MHz. We will use 1MHz to be safe.
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  SPI.transfer(0xFF); //Throw a dummy byte at the bus
  //Initialize VS1053 chip 
  delay(10);
  digitalWrite(MP3_RESET, HIGH); //Bring up VS1053
  //delay(10); //We don't need this delay because any register changes will check for a high DREQ

  //Mp3SetVolume(20, 20); //Set initial volume (20 = -10dB) LOUD
  Mp3SetVolume(40, 40); //Set initial volume (20 = -10dB) Manageable
  //Mp3SetVolume(80, 80); //Set initial volume (20 = -10dB) More quiet

  //Let's check the status of the VS1053
  int MP3Mode = Mp3ReadRegister(SCI_MODE);
  int MP3Status = Mp3ReadRegister(SCI_STATUS);
  int MP3Clock = Mp3ReadRegister(SCI_CLOCKF);

  Serial.print("SCI_Mode (0x4800) = 0x");
  Serial.println(MP3Mode, HEX);

  Serial.print("SCI_Status (0x48) = 0x");
  Serial.println(MP3Status, HEX);

  int vsVersion = (MP3Status >> 4) & 0x000F; //Mask out only the four version bits
  Serial.print("VS Version (VS1053 is 4) = ");
  Serial.println(vsVersion, DEC); //The 1053B should respond with 4. VS1001 = 0, VS1011 = 1, VS1002 = 2, VS1003 = 3

  Serial.print("SCI_ClockF = 0x");
  Serial.println(MP3Clock, HEX);

  //Now that we have the VS1053 up and running, increase the internal clock multiplier and up our SPI rate
  Mp3WriteRegister(SCI_CLOCKF, 0x60, 0x00); //Set multiplier to 3.0x

  //From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz. 
  //Internal clock multiplier is now 3x.
  //Therefore, max SPI speed is 5MHz. 4MHz will be safe.
  SPI.setClockDivider(SPI_CLOCK_DIV4); //Set SPI bus speed to 4MHz (16MHz / 4 = 4MHz)

  MP3Clock = Mp3ReadRegister(SCI_CLOCKF);
  Serial.print("SCI_ClockF = 0x");
  Serial.println(MP3Clock, HEX);

  //MP3 IC setup complete
  
  //Synth
  pinMode(pin_test,INPUT);
  pinMode(pin_save,INPUT);
  pinMode(pin_silent,INPUT);
  pinMode(pin_random,INPUT);
  randSeq();
  
  //leds-74HC595
  pinMode(latchPin, OUTPUT);
  digitalWrite(latchPin,LOW);
  
}

void loop(){
  for(int i=0;i<num_notes;i++){
    changeCurrSoundNum();
    changeNumNotes();
    
    if(iTesting()){
      playTestNote();
    }else {
      if(iSaving()){
        saveNote(i);
      }else if(iSilenting()){
        silentNote(i);
      }else if(iRandomizing()){
        randSeq();
      }
      playNote(i);
    }
    ledsDisp(i);
  }
}


//PlayMP3 pulls 32 byte chunks from the SD card and throws them at the VS1053
//We monitor the DREQ (data request pin). If it goes low then we determine if
//we need new data or not. If yes, pull new from SD card. Then throw the data
//at the VS1053 until it is full.
void playMP3(char* fileName) {

  uint8_t *mp3DataBuffer;
  int offset = 0;
  uint32_t size;

  //Serial.println("Start MP3 decoding");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File track = SD.open(fileName, FILE_READ);
  if (!track)
  {
    //Serial.print(fileName);
    //Serial.println(" not found");
    // don't do anything more:
    return;
  }
 
   size = track.size();
   
   mp3DataBuffer = (uint8_t *)malloc(size);
   if (!mp3DataBuffer)
   {
      //Serial.println("Failed to alloc mem for data buffer");
      return;
   }
  //Serial.println("Track open");

  offset = 0;
  while (offset < size)
  {
    int nbytes = size - offset;
    int ret;
    
    /* Read no more than one page at a time */
    if (nbytes > PAGE_SIZE)
      nbytes = PAGE_SIZE;

    ret = track.read(mp3DataBuffer+offset, nbytes);
    if (ret < 0)
    {    
      //Serial.print("Failed to read file, error: ");
      //Serial.println(ret);
      return;
    }
    
    offset += ret;
  }
  
   //Serial.print("Read whole file, size is: ");
   //Serial.println(size);

  /* Start feeding data to the VS1053 */
  offset  = 0;
  digitalWrite(MP3_XDCS, LOW); //Select Data
  while(offset < size && !getCommand()) {
    //Once DREQ is released (high) we now feed 32 bytes of data to the VS1053 from our SD read buffer
    while(!digitalRead(MP3_DREQ));

    SPI.transferBuffer(mp3DataBuffer + offset, NULL, (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : size - offset); // Send SPI bytes
    offset += CHUNK_SIZE;
    
    //getSynthInput();
  }
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data

  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating transfer is complete

  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  
  track.close(); //Close out this track
  free(mp3DataBuffer);

  //sprintf(errorMsg, "Track %s done!", fileName);
  //Serial.println(errorMsg);
}

//Write to VS10xx register
//SCI: Data transfers are always 16bit. When a new SCI operation comes in 
//DREQ goes low. We then have to wait for DREQ to go high again.
void Mp3WriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control

  //SCI consists of instruction byte, address byte, and 16-bit data word.
  SPI.transfer(0x02); //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
}

//Read the 16-bit value of a VS10xx register
unsigned int Mp3ReadRegister (unsigned char addressbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control

  //SCI consists of instruction byte, address byte, and 16-bit data word.
  SPI.transfer(0x03);  //Read instruction
  SPI.transfer(addressbyte);

  char response1 = SPI.transfer(0xFF); //Read the first byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  char response2 = SPI.transfer(0xFF); //Read the second byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete

  digitalWrite(MP3_XCS, HIGH); //Deselect Control

  int resultvalue = response1 << 8;
  resultvalue |= response2;
  return resultvalue;
}

//Set VS10xx Volume Register
void Mp3SetVolume(unsigned char leftchannel, unsigned char rightchannel){
  Mp3WriteRegister(SCI_VOL, leftchannel, rightchannel);
}

char getCommand(){
  return false;//this function is disabled
  
  if(Serial.available()){
    byte b=Serial.read();
    if(b=='s')
      return true;
    else 
      return false;
  }
}

boolean iTesting(){
  return !digitalRead(pin_test);
}
boolean iSaving(){
  return !digitalRead(pin_save); 
}
boolean iSilenting(){
  return !digitalRead(pin_silent); 
}
boolean iRandomizing(){
  //Serial.println(digitalRead(pin_random));
  return !digitalRead(pin_random);
}

void playNote(int note_num){
  sprintf(trackName, "track%03d.mp3", sequence[note_num]+1); //Splice the new file number into this file name
  playMP3(trackName); //Go play trackXXX.mp3

}
void playTestNote(){
  sprintf(trackName, "track%03d.mp3", curr_sound_num+1); //Splice the new file number into this file name
  playMP3(trackName); //Go play trackXXX.mp3

}
void saveNote(int note_num){
  sequence[note_num]=curr_sound_num;
}
void silentNote(int note_num){
  while(!digitalRead(pin_silent)){
    delay(10);
  }
  //sequence[note_num]=-1;
}

void displayVal(int v){
  Serial.print(v);
  Serial.print(", ");
}

void changeCurrSoundNum(){
  int raw=analogRead(knob_choose_pin);
  int currNum=raw/1023.0*num_sounds;
  
  curr_sound_num=currNum;
}
void changeNumNotes(){
  int raw=analogRead(knob_nn_pin);
  int numNotes=raw/1023.0*notes_len_max+1;
  
  //Serial.println(numNotes);
  num_notes=numNotes;
}
void randSeq(){
  for(int i=0;i<notes_len_max;i++){
    sequence[i]=(random()&0xff)/255.0*num_sounds;
  }
}


//leds-74HC595

void ledsDisp(int i){
    byte dataToSend = 1<<i;
      
    // setlatch pin low so the LEDs don't change while sending in bits
    digitalWrite(latchPin, LOW);
    // shift out the bits of dataToSend to the 74HC595
    //shiftOut(dataPin, clockPin, MSBFIRST, dataToSend);
    SPI.transfer(dataToSend);
    //set latch pin high- this sends data to outputs so the LEDs will light up
    digitalWrite(latchPin, HIGH);
}
