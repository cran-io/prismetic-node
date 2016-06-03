enum{OUTCURVE,MEASURING};
enum{OUTCURVE2,MEASURING2};
int mstatus=OUTCURVE;
int mstatus2=OUTCURVE2;
double sensorDistance=0.045;
const unsigned int MUESTRAS=10;
const unsigned int SAMPLE_TIME = 1;
const double MAXMEASURE=150;
const double TRIGGER_MAX=10;
const double TRIGGER_MIN=-10;
const unsigned int NSENSORS=2;
double prom=0;
unsigned int counter=0;

enum{MAX,MIN};

unsigned int sensordata[NSENSORS][MUESTRAS];
unsigned int dataFilter[NSENSORS];
double curve_Peak[NSENSORS];
double peak_Stamp[NSENSORS];
double newspeed=0;

#define filterSamples   13
int sensSmoothArray1 [filterSamples];
int smoothData1;

int sensSmoothArray2 [filterSamples];
int smoothData2;

int restaCurvas;

enum{IDLE_MIN,MEASURING_MIN,MIN_AQUIRED};
enum{IDLE_MAX,MEASURING_MAX,MAX_AQUIRED};

int minPeak=0;
int maxPeak=0;
int minPeak_Data=0;
int maxPeak_Data=0;
int minPeak_Stamp=0;
int maxPeak_Stamp=0;
int status_min=IDLE_MIN;
int status_max=IDLE_MAX;

boolean sensorMin=false;
boolean sensorMax=false;


unsigned int minPeakStamp;
unsigned int maxPeakStamp;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  curve_Peak[MIN]=MAXMEASURE;
  curve_Peak[MAX]=MAXMEASURE;
  dataFilter[0]=MAXMEASURE;
  dataFilter[1]=MAXMEASURE;
  for (int i=0;i<MUESTRAS;i++){
    sensordata[0][i]=MAXMEASURE;
    sensordata[1][i]=MAXMEASURE;
  }
}

double sTocm(unsigned int sensorValue){
  //return(sensorValue);
  return ( 10650.08 * pow(sensorValue,-0.935) - 10);
}

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

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

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

/*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}


void updateSensors(void){
    if (counter>=MUESTRAS){
      for (int i=0;i<MUESTRAS-1;i++){
        sensordata[0][i]=sensordata[0][i+1];
        sensordata[1][i]=sensordata[1][i+1];
      }
      counter=0;
    }else{
      sensordata[0][counter]=(int)sTocm(analogRead(A0));
          delay(SAMPLE_TIME);
      sensordata[1][counter]=(int)sTocm(analogRead(A1));
      
          delay(SAMPLE_TIME);
  
      counter++;
  }
  
  sort(sensordata[0],MUESTRAS);
  sort(sensordata[1],MUESTRAS);
  
  for (int i=0;i<MUESTRAS;i++){
      dataFilter[0]=dataFilter[0]+sensordata[0][i];
      dataFilter[1]=dataFilter[1]+sensordata[1][i];
   }

  dataFilter[0]/=MUESTRAS;
  dataFilter[1]/=MUESTRAS;
  /*
  Serial.print("data 0: ");
  Serial.println(dataFilter[0]);
  Serial.print("data 1: ");
  Serial.println(dataFilter[1]);*/
  
}




void loop() {
  // put your main code here, to run repeatedly:
  updateSensors();
  newspeed=sensorDistance/((peak_Stamp[MIN]-peak_Stamp[MAX])/1000);
  smoothData1 = digitalSmooth(dataFilter[0], sensSmoothArray1);
  smoothData2 = digitalSmooth(dataFilter[1], sensSmoothArray2);
  if (smoothData1>MAXMEASURE){
    smoothData1=MAXMEASURE;
  }
  if (smoothData2>MAXMEASURE){
    smoothData2=MAXMEASURE;
  }
 // Serial.println(smoothData1);
 // Serial.println(smoothData2);

  restaCurvas=smoothData1-smoothData2;
 // Serial.println(restaCurvas);
  
/*
  Serial.print("Data sensor 0: ");
  Serial.println(dataFilter[0]);
  
  Serial.print("Data sensor 1: ");
  Serial.println(dataFilter[1]);*/

  if (mstatus==OUTCURVE && restaCurvas<TRIGGER_MIN){
      mstatus=MEASURING;
      
      curve_Peak[MIN]=restaCurvas;
  }

  if(mstatus==MEASURING && restaCurvas==0){
    sensorMin=true;
    mstatus=OUTCURVE;
  }
  
  if (mstatus==MEASURING){
    if (curve_Peak[MIN]>restaCurvas){
      curve_Peak[MIN]=restaCurvas;
      peak_Stamp[MIN]=millis();
      Serial.print("Curve Peak min: ");
      Serial.println(curve_Peak[MIN]);
      Serial.print("timestamp min: ");
      Serial.println(peak_Stamp[MIN]);
      
    }  
  }


////////////////


  if (mstatus2==OUTCURVE2 && restaCurvas>TRIGGER_MAX){
      mstatus2=MEASURING2;
      /*Serial.print("Entro al status MEASURING, DATA FILTER VALUE: ");
      Serial.println(dataFilter[1]);*/
      curve_Peak[MAX]=restaCurvas;
  }

  if(mstatus2==MEASURING2 && restaCurvas==0){
    /*Serial.print("Salgo del status MEASURING, DATA FILTER VALUE: ");
      Serial.println(dataFilter[1]);*/
    sensorMax=true;
    mstatus2=OUTCURVE2;

  }
  
  if (mstatus2==MEASURING2){
    if (curve_Peak[MAX]<restaCurvas){
      curve_Peak[MAX]=restaCurvas;
      peak_Stamp[MAX]=millis();
      Serial.print("Curve Peak max: ");
      Serial.println(curve_Peak[MAX]);
      Serial.print("timestamp max: ");
      Serial.println(peak_Stamp[MAX]);

    }  
  }


  if (peak_Stamp[MAX]-peak_Stamp[MIN]>0){
    if (yaentro=false){
      Serial.println("Entre");
      yaentro=true;
    }else{
      
    }
    
  }else{
    Serial.println("Sali");
  }

  
 // Serial.println(mstatus);
/*

  Serial.print("Data sensor 0: ");
  Serial.println(dataFilter[0]);
  
  Serial.print("Data sensor 1: ");
  Serial.println(dataFilter[1]);*/
  
}

