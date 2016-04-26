//Sensor pinout
#define sens1 A0
#define sens2 A1
#define led1 4
#define led2 5
#define MUESTRAS 5

//Timing variables
long delaytimer = 0, timeOut3 = 0;

//Analog readings
int sen1antes = 0;
int sen2antes = 0;
int sen1 = 0;
int sen2 = 0;

//Detection parameters & variables
int RANGO = 175, HISTERESIS = 20, gente = 5, genteAntes = 0;
boolean ping1SS = false, ping2SS = false, ping1first = false, nose = false, nose2 = false, ping1entrada = false;
//para los 2 con aref 5v =175 esta ok
//aref 3v3 es mierdA

//Connection Libraries
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>

//Connection variables
RF24 radio(8, 9);                   // nRF24L01(+) radio attached using Getting Started board
RF24Network network(radio);          // Network uses that radio
const uint16_t this_node = 01;        // Address of our node in Octal format
const uint16_t other_node = 00;       // Address of the other node in Octal format
struct payload_t {                  // Structure of our payload
  unsigned char totalPeopleInside;
  signed char deltaPeople;
};
payload_t payload;
unsigned char EEPROMAddress = 0;

void setup() {
  // analogReference(EXTERNAL);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  delay(60);
  Serial.begin(9600);
  delaytimer = millis();

  //Connection setup
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);
  payload.totalPeopleInside = EEPROM.read(EEPROMAddress);
  payload.deltaPeople = 0;

  Serial.println("Starting sensor transceiver");
}

void loop() {
  if ((delaytimer - millis()) > 30) {
    readSensors();
    digitalWrite(led1, ((sen1antes > RANGO) && (sen1 > RANGO)));
    digitalWrite(led2, ((sen2antes > RANGO) && (sen2 > RANGO)));

    updateDetectionVariables();

    //user options
    if (Serial.available() > 0) {
      int input = Serial.read();
      switch (input) {
        case '0':
          asm("jmp 0x0000");
          break;
        case '5':
          gente = 5;
          break;
        default:
          Serial.println("No es una instruccion valida");
          break;
      }
    }

    sen1antes = sen1;
    sen2antes = sen2;

    delaytimer = millis();
  }

  //Connection update
  network.update();                          // Check the network regularly

  if (genteAntes != gente) {
    Serial.println(gente);
    payload.totalPeopleInside = gente;
    EEPROM.write(EEPROMAddress, payload.totalPeopleInside);

    //Post to Rx
    RF24NetworkHeader header(other_node);
    bool ok = network.write(header, &payload, sizeof(payload));
    Serial.print("Sending...");
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");

    genteAntes = gente;
  }

}

void readSensors() {
  int readings[2][MUESTRAS];
  sen1 = sen2 = 0;

  for (int i = 1; i < (MUESTRAS + 1); i++) {
    readings[0][(i - 1)] = analogRead(sens1);
    sen1 += readings[0][(i - 1)];
    readings[1][(i - 1)] = analogRead(sens2);
    sen2 += readings[1][(i - 1)];
  }

  sen1 /= MUESTRAS ;
  sen2 /= MUESTRAS ;
}

void updateDetectionVariables() {
  if (((sen1antes > RANGO) && (sen1 > RANGO)) && (ping1SS == false)) { //DETECCION DEL SENSOR 1
    ping1SS = true;
    if (ping2SS == false)
      ping1first = true;
  }

  if (((sen2antes > RANGO) && (sen2 > RANGO)) && (ping2SS == false) ) { //DETECCION DEL SENSR 2
    ping2SS = true;
  }


  if ((ping1SS == true) && (sen1antes < (RANGO - HISTERESIS)) && (sen1 < (RANGO - HISTERESIS)) && (ping2SS == true) && (sen2antes < (RANGO - HISTERESIS)) && (sen2 < (RANGO - HISTERESIS))) {
    //Set detection variables to default values
    ping1SS = false;
    ping2SS = false;
    ping1first = false;
    nose = false;
  }

  if (((ping2SS == true) && (ping1SS == true) && (nose == false))) {
    nose = true;
    nose2 = false;
    ping1SS = false;
    ping2SS = false;
    if (ping1first == true) {
      if (ping1entrada == true) {
        gente++ ;
      } else {
        gente--;
      }
    } else {
      if (ping1entrada == true) {
        gente--;
      } else {
        gente++;
      }
    }
    ping1first = false;
    gente = constrain(gente, 0, 9999999);
  }

  //SACAR PINGS EN TRUE SI NADIE PASA
  if (((ping1SS == true) || (ping2SS == true)) && (nose2 == false)  &&  (sen1antes < (RANGO - HISTERESIS))  &&  (sen1 < (RANGO - HISTERESIS))  &&  (sen2antes < (RANGO - HISTERESIS))  &&  (sen2 < (RANGO - HISTERESIS))  ) {
    nose2 = true;
    timeOut3 = millis();
  }


  if (((ping1SS == true) || (ping2SS == true) ) && (nose2 == true)) {
    if ((millis() - timeOut3) > 1000 ) {
      ping1SS = false;
      ping2SS = false;
      ping1first = false;
      nose = false;
      nose2 = false;
    }
  }
}

