/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 */

#include <SPI.h>
#include <EEPROM.h>
#include <MFRC522.h>
#include "DHT.h"
#include "Antirimbalzo.h"
#include "Proto485.h"

//#define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(x, ...)  Serial.print (x, ##__VA_ARGS__)
 #define DEBUG_PRINTLN(x, ...)  Serial.println (x, ##__VA_ARGS__)
#else
 #define DEBUG_PRINT(x, ...)
 #define DEBUG_PRINTLN(x, ...)  
#endif
#define DURATACLICKLUNGO 1200 // tempo pressione pulsante per click lungo = 2 secondi
#define TBLACKOUTPULSANTE 100 // tempo blackout pulsante dopo un click
#define TBLACKOUTPIR 3000
#define PULSANTE 4
#define PIR 3
#define TXENABLE 2
#define INTERVALLORITRASMISSIONTAG 1000

constexpr uint8_t RST_PIN = 9;          // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 10;         // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
Antirimbalzo swApricancello;
Antirimbalzo pir;
// se attivato il prossimo tag ricevuto viene memorizzato in eeprom 
bool memorizzaprossimotag=false;

volatile unsigned long int tiniziomemorizzazionetag,tultimaletturatemp;
DHT dht;
Proto485 comm(TXENABLE);


void setup() {
	Serial.begin(9600);		// Initialize serial communications with the PC
	SPI.begin();			// Init SPI bus
	mfrc522.PCD_Init();		// Init MFRC522
	//mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
  // aumenta la potenza
  mfrc522.PCD_SetRegisterBitMask(mfrc522.RFCfgReg, (0x07<<4));
	//Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  pinMode(TXENABLE, OUTPUT);
  pinMode(PULSANTE, INPUT_PULLUP);
  digitalWrite(TXENABLE, LOW);
  dht.setup(5,DHT::DHT22);
  swApricancello.cbInizioStatoOn=PulsanteApricancelloClick;
  pir.cbInizioStatoOn=pirAttivato;
  //pir.tPeriodoBlackOut=6000;
  DEBUG_PRINTLN("ok");    
  comm.cbElaboraComando=ElaboraComando;
}


void loop() {

	if ( mfrc522.PICC_IsNewCardPresent()) {
		ElaboraLetturaCard();
	}
  swApricancello.Elabora(digitalRead(PULSANTE)==LOW);
  pir.Elabora(digitalRead(PIR)==HIGH);
  if(Serial.available()) comm.ProcessaDatiSeriali(Serial.read());
  if( (millis() - tiniziomemorizzazionetag) > 20000) memorizzaprossimotag=false;
  if( (millis() - tultimaletturatemp) > 60000) {
    tultimaletturatemp=millis();
    dht.readSensor();
    uint32_t tmp=dht.Rtemperature;
    DEBUG_PRINT("temp="); 
    DEBUG_PRINT(tmp); 
    char uu[4];
    uu[0]=tmp & 0xff;
    uu[1]=tmp >> 8;
    tmp=dht.Rhumidity;
    DEBUG_PRINT(" hum="); 
    DEBUG_PRINTLN(tmp); 
    uu[2]=tmp & 0xff;
    uu[3]=tmp >> 8;
    comm.Tx('I',4,uu);
  }
  

}

void ElaboraLetturaCard() {
  static long int tultimoriconoscimento;
  if(!memorizzaprossimotag && ( (millis()-tultimoriconoscimento) < INTERVALLORITRASMISSIONTAG)) return;
  if ( ! mfrc522.PICC_ReadCardSerial()) return;
  //mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid));
  DEBUG_PRINT("letto tag: ");
  DEBUG_PRINT(mfrc522.uid.uidByte[0],HEX);
  DEBUG_PRINT(mfrc522.uid.uidByte[1],HEX);
  DEBUG_PRINT(mfrc522.uid.uidByte[2],HEX);
  DEBUG_PRINTLN(mfrc522.uid.uidByte[3],HEX);
  if(memorizzaprossimotag) {
    memorizzaCardNumber(mfrc522.uid.uidByte);
    return;
  }
  tultimoriconoscimento=millis();
  if(EsisteCorrispondenza(mfrc522.uid.uidByte)) comm.Tx('T',4,mfrc522.uid.uidByte);
}
  

void pirAttivato() {
  DEBUG_PRINTLN("pir");
  comm.Tx('B',0,0);
}


void PulsanteApricancelloClick() {
  DEBUG_PRINTLN("puls");
  comm.Tx('C',0,0);
}

void ElaboraComando(byte comando,byte *bytesricevuti,byte len) {
  switch(comando) {
    case 'F':
      memorizzaprossimotag=true;
      tiniziomemorizzazionetag=millis();
      DEBUG_PRINTLN("cmd F");
      break;
    case 'H':
      //ping
      comm.Tx('G',1,"D");
      break;
      
  }
}

void memorizzaCardNumber(byte* uidByte) {
  memorizzaprossimotag=false;
  if (EsisteCorrispondenza(uidByte)) {
      return;
  }
  // lo memorizza
  byte numerotagmemorizzati=EEPROM.read(0);
  EEPROM.write(1+numerotagmemorizzati*4, uidByte[0]);
  EEPROM.write(2+numerotagmemorizzati*4, uidByte[1]);
  EEPROM.write(3+numerotagmemorizzati*4, uidByte[2]);
  EEPROM.write(4+numerotagmemorizzati*4, uidByte[3]);
  EEPROM.write(0,++numerotagmemorizzati);
  DEBUG_PRINTLN("memorizzato");
}

bool EsisteCorrispondenza(byte* uidByte) {
  byte tmp[4];
  byte numerotagmemorizzati=EEPROM.read(0);
  DEBUG_PRINT("numtags: ");
  DEBUG_PRINTLN(numerotagmemorizzati);
  for(byte i=0;i<numerotagmemorizzati;i++) {
    tmp[0]=EEPROM.read( 1+i*4);
    tmp[1]=EEPROM.read( 2+i*4);
    tmp[2]=EEPROM.read( 3+i*4);
    tmp[3]=EEPROM.read( 4+i*4);
    /*
    DEBUG_PRINT(tmp[0],HEX);
    DEBUG_PRINT(tmp[1],HEX);
    DEBUG_PRINT(tmp[2],HEX);
    DEBUG_PRINT(tmp[3],HEX);
    DEBUG_PRINT(uidByte[0],HEX);
    DEBUG_PRINT(uidByte[1],HEX);
    DEBUG_PRINT(uidByte[2],HEX);
    DEBUG_PRINT(uidByte[3],HEX);
    */
    if(uidByte[0]==tmp[0] && uidByte[1]==tmp[1] && uidByte[2]==tmp[2] && uidByte[3]==tmp[3]) {
      DEBUG_PRINTLN("c'è");
      return true;
    }
  }
  DEBUG_PRINTLN("non c'è");
  return false;
}




