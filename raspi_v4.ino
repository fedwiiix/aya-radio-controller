#include <OneWire.h>
#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();
#include <IRremote.h>
int RECV_PIN = 4;
IRsend irsend;
IRrecv irrecv(RECV_PIN); 
decode_results results;
#include <SoftwareSerial.h>
SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"
#include "SPI.h"
BME280 capteur;


String message;
String cmd;

char  ser=0,val;
char flag,  led=0;
long code_radio;

int i;

/** Broche "DATA" du capteur  de temperature DHT22*/
const byte BROCHE_CAPTEUR = 5;
const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente dépassé
const byte DHT_CHECKSUM_ERROR = 2; // Données reçues erronées
 

const int ledRouge=7; 
const int ledVert=8; 
const int ledBleu=9;

void setup() {
  Serial.begin(9600);
  HC12.begin(9600);

  pinMode(ledRouge, OUTPUT);
  pinMode(ledVert, OUTPUT);
  pinMode(ledBleu, OUTPUT);

  pinMode(BROCHE_CAPTEUR, INPUT_PULLUP);
      
  mySwitch.enableReceive(0);   // pin 2
  mySwitch.enableTransmit(12); // pin 12
  irrecv.enableIRIn(); // on commence la réception ir   


//***********************************bmp280****************************************************
  while (!Serial) {
    // Attente de l'ouverture du port série pour Arduino LEONARDO
  }
  //configuration du capteur
  capteur.settings.commInterface = I2C_MODE; 
  capteur.settings.I2CAddress = 0x76;
  capteur.settings.runMode = 3; 
  capteur.settings.tStandby = 0;
  capteur.settings.filter = 0;
  capteur.settings.tempOverSample = 1 ;
  capteur.settings.pressOverSample = 1;
  capteur.settings.humidOverSample = 1;
 
  delay(10);  // attente de la mise en route du capteur. 2 ms minimum
  // chargement de la configuration du capteur
  capteur.begin();
}
void loop() {

//******************************************** reception serie    


if (HC12.available()) {        // If HC-12 has data
  //Serial.write(HC12.read());      // Send the data to Serial monitor
    /*val = HC12.read();
    if(val=='|'){
         command[ser]='\0';
         Serial.println(command);
         flag=1;
    }else{
      command[ser] = val;
      ser++;
    }*/
    message = HC12.readString();
    //Serial.println(message);
    cmd=message.substring(0,4);
    //Serial.println(cmd);
    flag=1;
    digitalWrite(ledVert,HIGH);
}


/*  
if( Serial.available() )  { 
    val = Serial.read();
    if(val=='|'){
         command[ser]='\0';
         Serial.println(command);
         flag=1;
    }else{
      command[ser] = val;
      ser++;
    }
}*/

//******************************************** reception radio

if (mySwitch.available()) {
  int RCvalue = mySwitch.getReceivedValue();
  if (RCvalue != 0) {

    HC12.print("{ \"Radio\":\"");
    HC12.print( mySwitch.getReceivedValue() );  
    HC12.println("\" }");
    //Serial.print("{ \"Radio\":\"");
    //Serial.print( mySwitch.getReceivedValue() );  
    //Serial.println("\" }");
  }
  digitalWrite(ledRouge,HIGH);
  delay(20);
  mySwitch.resetAvailable();
} else {
  digitalWrite(ledRouge,LOW);
}

//******************************************** reception ir

if (irrecv.decode(&results)) {
    HC12.print("{ \"IR\":\"");
    HC12.print(results.value, DEC);
    HC12.println("\" }");
    //Serial.print("{ \"IR\":\"");
    //Serial.print(results.value, DEC);
    //Serial.println("\" }");
    digitalWrite(ledBleu,HIGH);
    delay(20);
    irrecv.resume();
  }else{
   digitalWrite(ledBleu,LOW);
  }


  if(flag==1){ // Flag

//******************************************** emmission radio |R4331312081|

    if(cmd.equals("R433")){
      mySwitch.enableTransmit(12);
      code_radio = message.substring(4).toInt();
      //Serial.println(code_radio);
      mySwitch.send(code_radio , 24);
      HC12.print("FAIT");
      HC12.print(code_radio);
      
    }else if(cmd.equals("R315")){
      mySwitch.enableTransmit(6);
      code_radio = message.substring(4).toInt();
      //Serial.println(code_radio);
      mySwitch.send(code_radio , 24);
      HC12.print("FAIT");
      HC12.print(code_radio);
    
    }else if(cmd.equals("IRSE")){
      mySwitch.enableTransmit(6);
      code_radio = message.substring(4).toInt();
      //Serial.println(code_radio);
      for(i=0; i<3;i++){
        irsend.sendNEC(code_radio, 32);
      }
      HC12.print("FAIT");
      HC12.print(code_radio);
    }
    

//******************************************** Capteurs   |CAPTEUR|

    
    if(message.equals("CAPTEUR")){
      float temperature, humidity;

      if (readDHT22(BROCHE_CAPTEUR, &temperature, &humidity)==DHT_SUCCESS) {
         
        /* Affichage de la température et du taux d'humidité */

//***********************************bmp280****************************************************

        HC12.print("{ \"BMPTemperature\":\"");
        HC12.print(capteur.readTempC(), 2);
        HC12.print("\",\"BMPPressure\":\"");
        HC12.print(capteur.readFloatPressure()/10, 2);
        HC12.print("\",\"BMPHumidity\":\"");
        HC12.print(capteur.readFloatHumidity(), 2);
        
//***********************************lum280****************************************************

        float brightness=analogRead(A0);
        HC12.print("\",\"Brightness\":\"");
        HC12.print( (brightness*100/1024) , 2);
  
//***********************************dht22****************************************************

        HC12.print("\",\"Humidity\":\"");
        HC12.print(humidity, 2);
        HC12.print("\",\"Temperature\":\"");
        HC12.print(temperature, 2);
        HC12.println("\" }");
        /*Serial.print("{ \"Humidite\":\"");
        Serial.print(humidity, 2);
        Serial.print("\",\"Temperature\":\"");
        Serial.print(temperature, 2);
        Serial.println("\" }");*/
     
      }else{
        if(flag==2){
          HC12.print("{ \"BMPTemperature\":\"");
          HC12.print(capteur.readTempC(), 2);
          HC12.print("\",\"BMPPressure\":\"");
          HC12.print(capteur.readFloatPressure()/10, 2);
          HC12.print("\",\"BMPHumidity\":\"");
          HC12.print(capteur.readFloatHumidity(), 2);
                  
          float brightness=analogRead(A0);
          HC12.print("\",\"Brightness\":\"");
          HC12.print( (brightness*100/1024) , 2);
    
          HC12.println("\",\"Humidity\":\"0.00\",\"Temperature\":\"0.00\" }");
        }
      }
    }
      
    


    //******************************************** emmission radio HC-12    |HC12eau60|      
      if(cmd.equals("HC12")){
        HC12.print("|");    
        HC12.print(message.substring(4));
        HC12.println("|");
      }


    //******************************************** reinitialisation message
    
    message="";
    flag=0;
    delay(20);
    digitalWrite(ledVert,LOW);
    
  }
 
}

void ledRVBpwm(int pwmRouge, int pwmVert, int pwmBleu) { // reçoit valeur 0-255 par couleur
   //--- attention - avec une LED RGB anode commune : la LED s'allume sur niveau BAS !
 analogWrite(ledRouge, pwmRouge); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
 analogWrite(ledVert, pwmVert); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
 analogWrite(ledBleu, pwmBleu); // impulsion largeur voulue sur la broche 0 = 0% et 255 = 100% haut
}




byte readDHT22(byte pin, float* temperature, float* humidity) {
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 1, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS) 
    return ret;
  /* Calcul la vraie valeur de la température et de l'humidité */
  float fh = data[0];
  fh *= 256;
  fh += data[1];
  fh *= 0.1;
  *humidity = fh;
 
  float ft = data[2] & 0x7f;
  ft *= 256;
  ft += data[3];
  ft *= 0.1;
  if (data[2] & 0x80) {
    ft *= -1;
  }
  *temperature = ft;
  /* Ok */
  return DHT_SUCCESS;
}

/**
 * Fonction bas niveau permettant de lire la température et le taux d'humidité (en valeurs brutes) mesuré par un capteur DHTxx.
 */
byte readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout) {
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes
 
  /* Conversion du numéro de broche Arduino en ports / masque binaire "bas niveau" */
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddr = portModeRegister(port);   // Registre MODE (INPUT / OUTPUT)
  volatile uint8_t *out = portOutputRegister(port); // Registre OUT (écriture)
  volatile uint8_t *in = portInputRegister(port);   // Registre IN (lecture)
  
  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  /* Evite les problèmes de pull-up */
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   // Laisse le temps à la résistance de pullup de mettre la ligne de données à HIGH
 
  /* Réveil du capteur */
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); // Temps d'attente à LOW causant le réveil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un délai
  // de plus de 16 millisecondes ne donne pas un timing assez précis.
  
  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();
  
  /* Passage en écoute */
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT
 
  /* Attente de la réponse du capteur */
  timeout = 0;
  while(!(*in & bit)) { /* Attente d'un état LOW */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*in & bit) { /* Attente d'un état HIGH */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  /* Lecture des données du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i) {
 
    /* Attente d'un état LOW */
    unsigned long cycles_low = 0;
    while(!(*in & bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un état HIGH */
    unsigned long cycles_high = 0;
    while(*in & bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    /* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      data[i / 8] |= 1;
    }
  }
  
  /* Fin de la portion de code critique */
  interrupts();
 
  /*
   * Format des données :
   * [1, 0] = humidité en %
   * [3, 2] = température en degrés Celsius
   * [4] = checksum (humidité + température)
   */
   
  /* Vérifie la checksum */
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  else
    return DHT_SUCCESS; /* Pas d'erreur */
}
