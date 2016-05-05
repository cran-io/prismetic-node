//Sensor pinout
//CONSTANTES
const int SENSORES = 2;
const int MUESTRAS = 11;
const int MUESTRAS_PROMEDIADAS = 5;
const float TRIGGER = 60; //Cada unidad son 5mV aprox
const float HISTERESIS = 20; //Cada unidad son 5mV aprox
const unsigned int TRIGGER_DUR_MIN = 25; //ms
const unsigned int TRIGGER_DIFF_MAX = 500; //ms
const unsigned int TRIGGER_DELAY_MAX = 5000; //ms

//Timing variables
long readTimer = 0;

//VARIABLES
float sen1antes = 0, sen2antes = 0;
long sen1Timer = 0, sen2Timer = 0;
unsigned int sen1Duration = 0, sen2Duration = 0;
boolean ping1SS = false, ping2SS = false;
int gente = 0, genteAntes = 0;

//LECTURAS
unsigned int readings[SENSORES][MUESTRAS];
unsigned int readIndex = 0;
float readingsFloor[SENSORES];

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
  Serial.begin(9600);
  Serial.println("PRISMETIC by CRAN.IO");
  Serial.print("Starting...");
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //crear la network
  gente = EEPROM.read(EEPROMAddress); //inicializar la gente que hay adentro (guardado en EEPROM)
  payload.totalPeopleInside = gente;
  payload.deltaPeople = 0;
  Serial.println(gente);
  delay(60);                                                          //Warm-up del sensor
  
  for (int i = 1; i < MUESTRAS; i++)                                      //Init readings
    readings[0][i] = analogRead(A0);
  for (int i = 1; i < MUESTRAS; i++)
    readings[1][i] = analogRead(A1);
  readingsFloor[0] = analogRead(A0);
  readingsFloor[0] = analogRead(A1);
  
  readTimer = millis();
}

void loop() {
  if ((millis() - readTimer) > 1) {                                   //Adquisicion de datos cada 6ms, 5 muestras de cada sensor son 30ms, se ordenan de mayor a menor y se promedian las 3 del medio
    readings[0][readIndex] = analogRead(A0);
    readings[1][readIndex] = analogRead(A1);
    
    int i, j;
    unsigned int ordered[SENSORES][MUESTRAS];
    for (i = 1; i < MUESTRAS; i++)
      ordered[0][i] = readings[0][i];
    for (i = 1; i < MUESTRAS; i++)
      ordered[1][i] = readings[1][i];
    
    for (i = 1; i < MUESTRAS; i++) {                                //Ordeno el primer sensor de menor a mayor
      unsigned int tmp = ordered[0][i];
      for (j = i; j >= 1 && tmp < ordered[0][j - 1]; j--)
        ordered[0][j] = ordered[0][j - 1];
      ordered[0][j] = tmp;
    }

    for (i = 1; i < MUESTRAS; i++) {                                //Ordeno el segundo sensor de menor a mayor
      unsigned int tmp = ordered[1][i];
      for (j = i; j >= 1 && tmp < ordered[1][j - 1]; j--)
        ordered[1][j] = ordered[1][j - 1];
      ordered[1][j] = tmp;
    }

    float sen1ahora = 0, sen2ahora = 0;
    for (i = (MUESTRAS-MUESTRAS_PROMEDIADAS)/2; i < (MUESTRAS+MUESTRAS_PROMEDIADAS)/2; i++) {
      sen1ahora += ordered[0][i];
      sen2ahora += ordered[1][i];
    }
    sen1ahora /= (MUESTRAS_PROMEDIADAS);
    sen2ahora /=  (MUESTRAS_PROMEDIADAS);

    //Serial.println(sen2ahora);

    /*Serial.print("Sensor 1: ");
      Serial.print(sen1ahora);
      Serial.print(". Sensor 2: ");
      Serial.print(sen2ahora);
      Serial.println(".");*/

    if (ping1SS == false) {
      if (sen1Timer > 0) {
        if ((millis() - sen1Timer) > TRIGGER_DELAY_MAX) { //Si paso mas de TRIGGER_DELAY_MAX mato este sensor
          sen1Timer = 0;
        }
      }
      else if ((sen1ahora > (readingsFloor[0] + TRIGGER)) && (sen1antes > (readingsFloor[0] + TRIGGER))) { //DETECCION DEL SENSOR 1
        ping1SS = true;
        sen1Timer = millis();
        sen1Duration = 0;
      }
    }
    else if ((sen1antes < (readingsFloor[0] + TRIGGER - HISTERESIS)) && (sen1ahora < (readingsFloor[0] + TRIGGER - HISTERESIS))) { //BAJA DEL SENSOR 1
      ping1SS = false;
      sen1Duration = millis() - sen1Timer; //Duracion del pulso 1
    }

    if (ping2SS == false) {
      if (sen2Timer > 0) {
        if ((millis() - sen2Timer) > TRIGGER_DELAY_MAX) { //Si paso mas de TRIGGER_DELAY_MAX mato este sensor
          sen2Timer = 0;
        }
      }
      else if ((sen2ahora > (readingsFloor[1] + TRIGGER)) && (sen2antes > (readingsFloor[1] + TRIGGER))) { //DETECCION DEL SENSOR 2
        ping2SS = true;
        sen2Timer = millis();
      }
    }
    else if ((sen2antes < (readingsFloor[1] + TRIGGER - HISTERESIS)) && (sen2ahora < (readingsFloor[1] + TRIGGER - HISTERESIS))) { //BAJA DEL SENSOR 2
      ping2SS = false;
      sen2Duration = millis() - sen2Timer; //Duracion del pulso 2
    }

    if ((ping1SS == false ) && (ping2SS == false) && (sen1Timer > 0) && (sen2Timer > 0)) { //Si ya bajaron los sensores
      Serial.print("Sensor 1 - Timer:");
        Serial.print(sen1Timer);
        Serial.print(", Duration: ");
        Serial.print(sen1Duration);
        Serial.println(".");
        Serial.print("Sensor 2 - Timer: ");
        Serial.print(sen2Timer);
        Serial.print(", Duration: ");
        Serial.print(sen2Duration);
        Serial.println(".");
      if ((abs(sen1Timer - sen2Timer) < TRIGGER_DIFF_MAX) && (sen1Duration > TRIGGER_DUR_MIN) && (sen2Duration > TRIGGER_DUR_MIN)) {
        //Si la diferencia entre los sensores es menor que TRIGGER_DIFF_MAX y la duracion de los pulsos es mas que TRIGGER_DUR_MIN
        if ( sen1Timer < sen2Timer)
          gente++ ;
        else
          gente--;

        if (gente < 0)
          gente = 0;
      }
      sen1Timer = 0;
      sen2Timer = 0;
      sen1Duration = 0;
      sen2Duration = 0;
    }
    sen1antes = sen1ahora;
    sen2antes = sen2ahora;
    readingsFloor[0] = 0.99f*readingsFloor[0] + 0.01f*readings[0][readIndex];
    readingsFloor[1] = 0.99f*readingsFloor[1] + 0.01f*readings[1][readIndex];
    readIndex++;
    if (readIndex >= MUESTRAS) {
      readIndex = 0;
    }
    readTimer = millis();
  }

  //Connection update
  network.update();                          // Check the network regularly

  if (genteAntes != gente) {
    if (gente > genteAntes)
      Serial.print("ENTRADA.");
    else
      Serial.print("SALIDA.");
    Serial.print("TOTAL: ");
    Serial.println(gente);
    payload.totalPeopleInside = gente;
    EEPROM.write(EEPROMAddress, gente);

    //Post to Rx
    RF24NetworkHeader header(other_node);
    bool ok = network.write(header, &payload, sizeof(payload));
    Serial.print("Sending...");
    if (ok) {
      genteAntes = gente;
      Serial.println("ok.");
    }
    else {
      Serial.println("failed.");
    }
    genteAntes = gente;
  }

  if (Serial.available() > 0) {                       //opciones por serial para resetear o establecer la gente en 5 para pruebas
    int input = Serial.read();
    switch (input) {
      case '0':
        asm("jmp 0x0000");
        break;
      case '5':
        gente = genteAntes = 5;
        Serial.println("SET TO 5");
        break;
      default:
        Serial.println("No es una instruccion valida");
        break;
    }
  }
}

