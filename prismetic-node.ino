//Sensor pinout
//CONSTANTES
const int SENSORES = 2;
const int MUESTRAS = 5;
const float TRIGGER = 800; //mV
const float HISTERESIS = 300; //mV

//Timing variables
long readTimer = 0, timeOut3 = 0;

//VARIABLES
float sen1antes = 0, sen2antes = 0, aux, sen1, sen2, sen1mV, sen2mV;
boolean ping1first = false, ping1SS = false, ping2SS = false, nose = false, nose2 = false, ping1entrada = false;
int gente = 0, genteAntes = 0;

//LECTURAS
unsigned int readings[SENSORES][MUESTRAS];
unsigned int readIndex = 0;

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
  delay(60);                                                          //Warm-up del sensor
  Serial.begin(9600);
  delaytimer = millis();                                              //inicializar timer de muestreo
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);                                       //crear la network
  payload.totalPeopleInside = EEPROM.read(EEPROMAddress);             //inicializar la gente que hay adentro (guardado en EEPROM)
  payload.deltaPeople = 0;
  Serial.println("PRISMETIC by CRAN.IO");
  Serial.print("Starting...");
  Serial.println(gente);

}

void loop() {
  if ((readTimer - millis()) > 6) {                                   //Adquisicion de datos cada 6ms, 5 muestras de cada sensor son 30ms, se ordenan de mayor a menor y se promedian las 3 del medio
    readings[0][readIndex] = analogRead(A0);
    readings[1][readIndex] = analogRead(A1);
    readIndex++;
    
    if(readIndex >= MUESTRAS){

      unsigned int aux;
      for (int i = 0; i < (MUESTRAS - 1); i++) {
        if (readings[1][i] > readings[1][i + 1])
          aux = readings[1][i];
        readings[1][i] = readings[1][i + 1];
        readings[1][i + 1] = aux;
      }

    for (int i = 0; i < (MUESTRAS - 1); i++) {
      if (readings[0][i] > readings[0][i + 1])
        aux = readings[0][i];
      readings[0][i] = readings[0][i + 1];
      readings[0][i + 1] = aux;
    }

    for (int i = 1; i < (MUESTRAS - 1); i++) {
      sen1 += readings[0][i];
      sen2 += readings[1][i];
    }
    sen1 /= (MUESTRAS - 2);
    sen2 /=  (MUESTRAS - 2);

    float sen1mV = ((5 * sen1) / 1024);                                        // Pasaje a mV de los datos obtenidos
    sen1mV *= 1000;

    float sen2mV = ((5 * sen2) / 1024);
    sen2mV *= 1000;

    if (((sen1antes > TRIGGER) && (sen1mV > TRIGGER)) && (ping1SS == false)) { //DETECCION DEL SENSOR 1
      ping1SS = true;
      if (ping2SS == false)
        ping1first = true;
    }

    if (((sen2antes > TRIGGER) && (sen2mV > TRIGGER)) && (ping2SS == false) ) { //DETECCION DEL SENSR 2
      ping2SS = true;
    }
    if ((ping1SS == true) && (sen1antes < (TRIGGER - HISTERESIS)) && (sen1mV < (TRIGGER - HISTERESIS)) && (ping2SS == true) && (sen2antes < (TRIGGER - HISTERESIS)) && (sen2mV < (TRIGGER - HISTERESIS))) {
      //Set detection variables to default values
      ping1SS = false;
      ping2SS = false;
      ping1first = false;
      nose = false;
    }

    if (((ping2SS == true) && (ping1SS == true) && (nose == false))) {          //sumar o restar una persona segun que sensor sea la entrada y cual detecto primero a la persona/objeto
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

    //Volver a false los ping1SS y ping2SS si se activa uno y el otro no
    if (((ping1SS == true) || (ping2SS == true)) && (nose2 == false)  &&  (sen1antes < (TRIGGER - HISTERESIS))  &&  (sen1mV < (TRIGGER - HISTERESIS))  &&  (sen2antes < (TRIGGER - HISTERESIS))  &&  (sen2mV < (TRIGGER - HISTERESIS))  ) {
      nose2 = true;
      timeOut3 = millis();
    }
    if (((ping1SS == true) || (ping2SS == true) ) && (nose2 == true)) {
      if ((millis() - timeOut3) > 1000 ) {                                  //Tiempo maximo de diferencia entre la activacion de un sensor y el otro para considerar el paso de una persona o no
        ping1SS = false;
        ping2SS = false;
        ping1first = false;
        nose = false;
        nose2 = false;
      }
    }
    sen1antes = sen1mV;
    sen2antes = sen2mV;
    gente = constrain(gente, 0, 9999999);      //constrain para que no haya gente negativa adentro, modificar en el futuro


    delaytimer = millis();
  }

  //Connection update
  network.update();                          // Check the network regularly

  if (genteAntes < gente) {
    Serial.print("ENTRADA. TOTAL: ");
    Serial.println(gente);
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
  else if (genteAntes > gente) {                                      //diferenciacion entre entrada o salida, imprimir en serial y enviar por RF
    Serial.print("SALIDA. TOTAL: ");
    Serial.println(gente);
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

