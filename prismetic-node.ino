
//Connection
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>

//Sensor pinout
const unsigned int SENSORES = 2;
const unsigned int MUESTRAS = 31;
const unsigned int MUESTRAS_PROMEDIADAS = 5;
const unsigned int RANGE_MASK = 3;
const float TRIGGER = 60; //Cada unidad son 5mV aprox
const float HISTERESIS = 20; //Cada unidad son 5mV aprox
const unsigned int TRIGGER_DUR_MIN = 25; //ms
const unsigned int TRIGGER_DIFF_MAX = 500; //ms
const unsigned int TRIGGER_DELAY_MAX = 5000; //ms

//Timing variables
const unsigned int MUESTRAS_PERIOD = 1; //1ms
unsigned long readTimer = 0;
const unsigned int POST_PERIOD = 500;
const unsigned int POST_RETRY = 50;
unsigned long postTimer = 0;

//Sensor variables
float sen1antes = 0, sen2antes = 0;
long sen1Timer = 0, sen2Timer = 0;
unsigned int sen1Duration = 0, sen2Duration = 0;
boolean ping1SS = false, ping2SS = false;

//Reading variables
unsigned int readings[SENSORES][MUESTRAS];
unsigned int timeStamps[SENSORES][MUESTRAS];
unsigned int minSample[SENSORES];
unsigned int readIndex = 0;
float readingsFloor[SENSORES];
bool newData=false;

//Connection variables
RF24 radio(8, 9);                   // nRF24L01(+) radio attached using Getting Started board
RF24Network network(radio);          // Network uses that radio
const uint16_t this_node = 02;        // Address of our node in Octal format
const uint16_t base_node = 00;       // Address of the other node in Octal format
struct payload_t {                  // Structure of our payload
  long totalPeopleInside;
  int peopleIn;
  int peopleOut;
};
payload_t payload;
unsigned char EEPROMAddress = 0;

//Leds
#define LED_CONN 2 //Pin 2
#define LED_COUNT 3 //Pin 3
#define LED_BLINK_PERIOD_ONLINE 2500 //ms
#define LED_BLINK_PERIOD_OFFLINE 250 //ms
bool online=false;


//Max filter (colo)
enum {MAX,MIN};
double dis1ahora;
double dis2ahora;
enum{OUTCURVE,READING};
int mstatus=OUTCURVE;
double sensorDistance=0.045;
const double MAX_MEASURE=150;
const unsigned int NSENSORS=2;
const double TRIGGER_PEAK=10;
const unsigned int MAX_PEAK_DIF=1800;

double curve_Peak[NSENSORS];
unsigned int peak_Stamp[NSENSORS];
unsigned int reading_Stamp=0;
int lastTime_dif=0;
int lastMeasure=0;
const int MAX_SPEED=6;

double newspeed=0;

#define filterSamples   20
int sensSmoothArray1 [filterSamples];
int smoothData1;
const unsigned int TIME_INTERPEOPLE =500;

int sensSmoothArray2 [filterSamples];
int smoothData2;

int restaCurvas;


void sort(unsigned int a[],int size){
  for(int i=0; i<(size-1);i++){
    for (int o=0;o<(size-(i+1));o++){
      if(a[o]>a[o+1]){
        int t= a[o];
        a[o]=a[o+1];
        a[o+1]=t;
      }
    }
  }
}


double sTocm(unsigned int sensorValue){
  return ( 10650.08 * pow(sensorValue,-0.935) - 10);
}


void setup() {
  Serial.begin(57600);
  //Serial.println("PRISMETIC by CRAN.IO");
  //Serial.print("Starting... ");
  pinMode(LED_COUNT,OUTPUT);
  pinMode(LED_CONN,OUTPUT);
  digitalWrite(LED_COUNT,LOW);
  digitalWrite(LED_CONN,LOW);
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  network.begin(90, this_node);  //crear la network
  payload.totalPeopleInside = EEPROM.read(EEPROMAddress); //inicializar la gente que hay adentro (guardado en EEPROM)
  payload.peopleIn = 0;
  payload.peopleOut = 0;
  //Serial.print("TOTAL: ");
  //Serial.println(payload.totalPeopleInside);
  delay(60);                                                          //Warm-up del sensor
  for (int i = 1; i < MUESTRAS; i++){                                 //Init readings
    readings[0][i] = analogRead(A0);
    readings[1][i] = analogRead(A1);
  /*  Serial.print("Reading 0: ");
    Serial.println(sTocm(readings[0][i]));
    Serial.print("Reading 1: ");
    Serial.println(sTocm(readings[1][i]));*/
    delay(100);
  }

  sort(readings[0],MUESTRAS);
  sort(readings[1],MUESTRAS);

  //Serial.print("Trigger: ");
  //Serial.println(sTocm(TRIGGER));
  
  for (int i = 1+RANGE_MASK; i < MUESTRAS-RANGE_MASK; i++){                                 //Init floor
    readingsFloor[0] += readings[0][i];
    readingsFloor[1] += readings[1][i];
    
    //Serial.print("Order Reading 0: ");
    //Serial.println(sTocm(readings[0][i]));
    //Serial.print("Order Reading 1: ");
    //Serial.println(sTocm(readings[1][i]));
   
  }
  readingsFloor[0] /= MUESTRAS-RANGE_MASK*2;
  readingsFloor[1] /= MUESTRAS-RANGE_MASK*2;

  readingsFloor[0]=(readingsFloor[0]+readingsFloor[1])/2;
  readingsFloor[1]=readingsFloor[0];
  Serial.println("Average floor");
  Serial.print("Readings Floor 0: ");
  Serial.println(sTocm(readingsFloor[0]));
  Serial.print("Readings Floor 1: ");
  Serial.println(sTocm(readingsFloor[1]));
  //Serial.print("Readings Floor 0: ");
  //Serial.println((readingsFloor[0]));
  //Serial.print("Readings Floor 1: ");
  //Serial.println((readingsFloor[1]));
  Serial.println("-------------------------------");
  Serial.println("-------------------------------");
  Serial.println("Node ready");
  Serial.println("-------------------------------");
  Serial.println("-------------------------------");
}




void set_Peak_Dif(int diference){
  lastTime_dif=diference;
}

int get_Peak_dif(void){
  return(lastTime_dif);
}

void updatePeaks(){
  restaCurvas=dis1ahora-dis2ahora;
  if (restaCurvas>99){
    restaCurvas=0;
  }


  if (mstatus == OUTCURVE && (restaCurvas   > TRIGGER_PEAK || restaCurvas < -TRIGGER_PEAK) && millis()-lastMeasure>TIME_INTERPEOPLE){
    Serial.println("Estamos dentro del rango de medicion");
    mstatus=READING;
    reading_Stamp=millis();
    curve_Peak[MIN]=0;
    curve_Peak[MAX]=0;
    set_Peak_Dif(0);
  }
    unsigned int realtime=millis();
  /*Serial.println(restaCurvas);
  Serial.println(realtime);*/
  if (mstatus==READING && (realtime-reading_Stamp)<MAX_PEAK_DIF){
     /*if ((restaCurvas < -TRIGGER_PEAK) ||  (restaCurvas > TRIGGER_PEAK)){ 
        Serial.println(restaCurvas);
        Serial.println(realtime);
     }*/
    if (curve_Peak[MIN]>restaCurvas && restaCurvas < -TRIGGER_PEAK){
      curve_Peak[MIN] = restaCurvas;
      peak_Stamp[MIN] = millis();    
     // Serial.print("Midiendo minimo: ");  
     // Serial.println(curve_Peak[MIN]);
    }
    if (curve_Peak[MAX] < restaCurvas && restaCurvas > TRIGGER_PEAK){
      curve_Peak[MAX] = restaCurvas;
      peak_Stamp[MAX] = millis();     
     // Serial.print("Midiendo maximo: ");  
     // Serial.println(curve_Peak[MAX]); 
    }
  }

  if (mstatus==READING && (restaCurvas < TRIGGER_PEAK && restaCurvas > -TRIGGER_PEAK) && curve_Peak[MIN] !=0 && curve_Peak[MAX]!=0){
    mstatus=OUTCURVE;
    curve_Peak[MIN]=0;
    curve_Peak[MAX]=0;
    Serial.println("Medicion completa");
    lastMeasure=millis();
    set_Peak_Dif(peak_Stamp[MAX]-peak_Stamp[MIN]);
    if (sensorDistance*8000/get_Peak_dif()< MAX_SPEED && sensorDistance*8000/get_Peak_dif() > -MAX_SPEED)
      countPeople();
  }

  if (mstatus ==READING &&( (realtime-reading_Stamp) > MAX_PEAK_DIF)){    
    mstatus=OUTCURVE;
  //  Serial.println("Demasiado tiempo entre picos, probable error");
    reading_Stamp=millis();
    curve_Peak[MIN]=0;
    curve_Peak[MAX]=0;
    peak_Stamp[MIN]=0;
    peak_Stamp[MAX]=0;
    set_Peak_Dif(0);
  }
  
}


int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // //Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // //Serial.print(sorted[j]); 
    // //Serial.print("   "); 
  }


  return total / k;    // divide by number of samples
}

void countPeople(){

        if ( get_Peak_dif()>0){
          payload.peopleIn++ ;
          payload.totalPeopleInside++;
          
          newData=true;
        }
        else{
          payload.peopleOut++;
          payload.totalPeopleInside--;
          newData=true;
        }
        if(payload.totalPeopleInside > 0)
          EEPROM.write(EEPROMAddress, payload.totalPeopleInside);
      sen1Timer = 0;
      sen2Timer = 0;
      sen1Duration = 0;
      sen2Duration = 0;
      Serial.print("Person speed: ");
      Serial.println(sensorDistance*8000/get_Peak_dif());
}



void loop() {
  if (millis() > readTimer) {                                   //Adquisicion de datos cada 1ms, 11 muestras de cada sensor son 11ms, se ordenan de mayor a menor y se promedian las 5 del medio
    readings[0][readIndex] = analogRead(A0);
    readings[1][readIndex] = analogRead(A1);
    
    int i, j;

    
    unsigned int ordered[SENSORES][MUESTRAS];
    for (i = 1; i < MUESTRAS; i++){
      
      ordered[0][i] = readings[0][i];
    }
    
    for (i = 1; i < MUESTRAS; i++){
      
      ordered[1][i] = readings[1][i];
    }


    
    for (i = 1; i < MUESTRAS; i++) {                                //Ordeno el primer sensor de menor a mayor y tomo el maximo
      unsigned int tmp = ordered[0][i];
      for (j = i; j >= 1 && tmp < ordered[0][j - 1]; j--)
        ordered[0][j] = ordered[0][j - 1];
      ordered[0][j] = tmp;
    }

    for (i = 1; i < MUESTRAS; i++) {                                //Ordeno el segundo sensor de menor a mayor y tomo el maximo
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

    dis1ahora=sTocm(sen1ahora);
    dis2ahora=sTocm(sen2ahora);
    
    if (dis1ahora>MAX_MEASURE){
      dis1ahora=MAX_MEASURE;
    }
    if (dis2ahora>MAX_MEASURE){
      dis2ahora=MAX_MEASURE;
    }
    

    dis1ahora = digitalSmooth(dis1ahora, sensSmoothArray1);
    dis2ahora = digitalSmooth(dis2ahora, sensSmoothArray2);
    
    updatePeaks();
      
    
    sen1antes = sen1ahora; //1 valor de historial
    sen2antes = sen2ahora;
    
    readingsFloor[0] = 0.99f*readingsFloor[0] + 0.01f*readings[0][readIndex]; //Este es un promedio que se actualiza muy lento
    readingsFloor[1] = 0.99f*readingsFloor[1] + 0.01f*readings[1][readIndex];
    
    readIndex++;  //Arreglo circular
    if (readIndex >= MUESTRAS) {
      readIndex = 0;
    }
    readTimer = millis() + MUESTRAS_PERIOD;

    if(ping1SS || ping2SS){
      digitalWrite(LED_COUNT, HIGH);
    }
    else{
      digitalWrite(LED_COUNT, LOW);
    }
  }

  //Connection update
  network.update();                          // Check the network regularly

  if (millis() > postTimer && newData) {
    online = post();
    if(online)
      postTimer = millis() + POST_PERIOD;
    else
      postTimer = millis() + POST_RETRY;
  }

  unsigned int ledBlinkPeriod = online?LED_BLINK_PERIOD_ONLINE:LED_BLINK_PERIOD_OFFLINE;
  if((millis()%ledBlinkPeriod)>(ledBlinkPeriod/2)){
    digitalWrite(LED_CONN, HIGH);
  }
  else{
    digitalWrite(LED_CONN, LOW);
  }

  if (Serial.available() > 0) {                       //opciones por serial para resetear o establecer la gente en 5 para pruebas
    int input = Serial.read();
    switch (input) {
      case '0':
        resetPeople();
        break;
      case 'p':
        if (newData){
          post();
        }
        break;
      default:
        //Serial.println("No es una instrucción valida");
        break;
    }
  }
}

bool post(){
  //Post to Rx
  RF24NetworkHeader header(base_node);
  bool success = network.write(header, &payload, sizeof(payload));
  //Serial.print("Sending... ");
  if (success) {
    Serial.println("Payload sent");
    Serial.print("People in: ");
    Serial.println(payload.peopleIn);
    Serial.print("People out: ");
    Serial.println(payload.peopleOut);
    payload.peopleIn = 0;
    payload.peopleOut = 0;
    Serial.println("post OK.");
    newData=false;
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
 // Serial.println("RESET PEOPLE TO 0");
}

