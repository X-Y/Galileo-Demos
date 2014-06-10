
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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
uint8_t *mp3DataBuffer;

char errorMsg[100]; //This is a generic array used for sprintf of error messages
 
boolean iPlaying=false;
boolean iStart=false;

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
 
 
int status = WL_IDLE_STATUS;
char ssid[] = "NETGEAR-32E"; //  your network SSID (name) 
char pass[] = "E44A3DB29B";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void setup() {
  pinMode(MP3_DREQ, INPUT_FAST);
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
  
  initUdp();
  
}
 
void loop(){
  getCommand();
  if(iStart){
    iStart=false;
    playMP3(packetBuffer);
  }
}
 
 
//PlayMP3 pulls 32 byte chunks from the SD card and throws them at the VS1053
//We monitor the DREQ (data request pin). If it goes low then we determine if
//we need new data or not. If yes, pull new from SD card. Then throw the data
//at the VS1053 until it is full.
void playMP3(char* fileName) {
 
  int offset = 0;
  uint32_t size;
 
  Serial.println("Start MP3 decoding");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  track = SD.open(fileName, FILE_READ);
  if (!track)
  {
    Serial.print(fileName);
    Serial.println(" not found");
    // don't do anything more:
    return;
  }
 
   size = track.size();
   
   mp3DataBuffer = (uint8_t *)malloc(size);
   if (!mp3DataBuffer)
   {
      Serial.println("Failed to alloc mem for data buffer");
      return;
   }
  Serial.println("Track open");
 
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
      Serial.print("Failed to read file, error: ");
      Serial.println(ret);
      return;
    }
    
    offset += ret;
  }
  
   Serial.print("Read whole file, size is: ");
   Serial.println(size);
 
  /* Start feeding data to the VS1053 */
  offset  = 0;
  digitalWrite(MP3_XDCS, LOW); //Select Data
  int c=0;
  int d=0;
  while(offset < size && !iStart) {
    c++;
    if(c>500){
      d++;
      Serial.print(".");
      if(d>55){
        Serial.println();
        d=0;
      }
      getCommand();
      c=0;
    }
    //Once DREQ is released (high) we now feed 32 bytes of data to the VS1053 from our SD read buffer
    while(!digitalRead(MP3_DREQ));
 
    SPI.transferBuffer(mp3DataBuffer + offset, NULL, (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : size - offset); // Send SPI bytes
    offset += CHUNK_SIZE;
    
    //getSynthInput();
  }
 
  endMP3();
  sprintf(errorMsg, "Track %s done!", fileName);
  Serial.println(errorMsg);
}
void endMP3(){
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
 
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating transfer is complete
 
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  
  track.close(); //Close out this track
  free(mp3DataBuffer);
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
  //return false;//this function is disabled
  
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {   
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer,255);
    if (len >0) packetBuffer[len]=0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    //playMP3(packetBuffer);
    iStart=true;
    
    
    // send a reply, to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
   }
}
 
void playNote(int note_num){
  sprintf(trackName, "track%03d.mp3", note_num); //Splice the new file number into this file name
  Serial.println(trackName);
  
  playMP3(trackName); //Go play trackXXX.mp3
 
}

/*
*
*  Udp connection
*
*/

void initUdp(){
    // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 

  String fv = WiFi.firmwareVersion();
  if( fv != "1.1.0" )
    Serial.println("Please upgrade the firmware");
  
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid,pass);
  
    // wait 10 seconds for connection:
    delay(3000);
  } 
  Serial.println("Connected to wifi");
  printWifiStatus();
  
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);  

}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

