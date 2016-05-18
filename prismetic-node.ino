//Connection
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>

//Sensor pinout
const unsigned int SENSORES = 2;
const unsigned int MUESTRAS = 11;
const unsigned int MUESTRAS_PROMEDIADAS = 5;
const float TRIGGER = 60; //Cada unidad son 5mV aprox
const float HISTERESIS = 20; //Cada unidad son 5mV aprox
const unsigned int TRIGGER_DUR_MIN = 25; //ms
const unsigned int TRIGGER_DIFF_MAX = 500; //ms
const unsigned int TRIGGER_DELAY_MAX = 5000; //ms

//Timing variables
const unsigned int MUESTRAS_PERIOD = 1; //1ms
unsigned long readTimer = 0;
const unsigned int POST_PERIOD = 10000;
const unsigned int POST_RETRY = 5000;
unsigned long postTimer = 0;

//Sensor variables
float sen1antes = 0, sen2antes = 0;
long sen1Timer = 0, sen2Timer = 0;
unsigned int sen1Duration = 0, sen2Duration = 0;
boolean ping1SS = false, ping2SS = false;

//Reading variables
unsigned int readings[SENSORES][MUESTRAS];
unsigned int readIndex = 0;
float readingsFloor[SENSORES];

//Connection variables
RF24 radio(8, 9);                   // nRF24L01(+) radio attached using Getting Started board
RF24Network network(radio);          // Network uses that radio
const uint16_t this_node = 01;        // Address of our node in Octal format
const uint16_t other_node = 00;       // Address of the other node in Octal format
struct payload_t {                  // Structure of our payload
  long totalPeopleInside;
  int peopleIn;
  int peopleOut;
};
payload_t payload;
unsigned char EEPROMAddress = 0;

void setup() {
  Serial.begin(57600);
  Serial.println("PRISMETIC by CRAN.IO");
  Serial.print("Starting... ");
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  network.begin(90, this_node);  //crear la network
  payload.totalPeopleInside = EEPROM.read(EEPROMAddress); //inicializar la gente que hay adentro (guardado en EEPROM)
  payload.peopleIn = 0;
  payload.peopleOut = 0;
  Serial.print("TOTAL: ");
  Serial.println(payload.totalPeopleInside);
  delay(60);                                                          //Warm-up del sensor
  for (int i = 1; i < MUESTRAS; i++){                                 //Init readings
    readings[0][i] = analogRead(A0);
    readings[1][i] = analogRead(A1);
    delay(10);
  }
  for (int i = 1; i < MUESTRAS; i++){                                 //Init floor
    readingsFloor[0] += readings[0][i];
    readingsFloor[1] += readings[1][i];
  }
  readingsFloor[0] /= MUESTRAS;
  readingsFloor[1] /= MUESTRAS;
}

void loop() {
  if (millis() > readTimer) {                                   //Adquisicion de datos cada 1ms, 11 muestras de cada sensor son 11ms, se ordenan de mayor a menor y se promedian las 5 del medio
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
    for (i = (MUESTRAS-MUESTRAS_PROMEDIADAS)/2; i < (MUESTRAS+MUESTRAS_PROMEDIADAS)/2; i++) { //Promedio de las muestras del medio
      sen1ahora += ordered[0][i];
      sen2ahora += ordered[1][i];
    }
    sen1ahora /= (MUESTRAS_PROMEDIADAS);
    sen2ahora /=  (MUESTRAS_PROMEDIADAS);

    //Serial.println(sen2ahora);

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
        if ( sen1Timer < sen2Timer){
          payload.peopleIn++ ;
          payload.totalPeopleInside++;
          Serial.print("IN. TOTAL: ");
          Serial.println(payload.totalPeopleInside);
        }
        else{
          payload.peopleOut++;
          payload.totalPeopleInside--;
          Serial.print("OUT. TOTAL: ");
          Serial.println(payload.totalPeopleInside);
        }
        if(payload.totalPeopleInside > 0)
          EEPROM.write(EEPROMAddress, payload.totalPeopleInside);
      }
      sen1Timer = 0;
      sen2Timer = 0;
      sen1Duration = 0;
      sen2Duration = 0;
    }
    
    sen1antes = sen1ahora; //1 valor de historial
    sen2antes = sen2ahora;
    
    readingsFloor[0] = 0.99f*readingsFloor[0] + 0.01f*readings[0][readIndex]; //Este es un promedio que se actualiza muy lento
    readingsFloor[1] = 0.99f*readingsFloor[1] + 0.01f*readings[1][readIndex];
    
    readIndex++;  //Arreglo circular
    if (readIndex >= MUESTRAS) {
      readIndex = 0;
    }
    readTimer = millis() + MUESTRAS_PERIOD;
  }

  //Connection update
  network.update();                          // Check the network regularly

  if (millis() > postTimer) {
    bool success = post();
    if(success)
      postTimer = millis() + POST_PERIOD;
    else
      postTimer = millis() + POST_RETRY;
  }

  if (Serial.available() > 0) {                       //opciones por serial para resetear o establecer la gente en 5 para pruebas
    int input = Serial.read();
    switch (input) {
      case '0':
        resetPeople();
        break;
      case 'p':
        post();
        break;
      default:
        Serial.println("No es una instrucción valida");
        break;
    }
  }
}

bool post(){
  //Post to Rx
  RF24NetworkHeader header(other_node);
  bool success = network.write(header, &payload, sizeof(payload));
  Serial.print("Sending... ");
  if (success) {
    payload.peopleIn = 0;
    payload.peopleOut = 0;
    Serial.println("post OK.");
  }
  else {
    Serial.println("post Failed.");
  }
  return success;
}

void resetPeople(){
  payload.totalPeopleInside = 0;
  payload.peopleIn = 0;
  payload.peopleOut = 0;
  EEPROM.write(EEPROMAddress, payload.totalPeopleInside);
  Serial.println("RESET PEOPLE TO 0");
}

