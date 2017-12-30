/* Version 5
  With LED driving codes and filters
  RedLED: pin 22; IRLED: pin 23
  Red input: pin A0; IR input: pin A1
  sampling frequency ~80Hz
  Period of analysis = 4 sec = 320 sampling points
  AC channel filters: bandpass, 0.4Hz-3.8Hz (24-228bpm),  order=4
  DC channel filters: lowpass, cutoff@ 0.5Hz, order=2
*/
#include "myfilters.h"
#include <SPI.h>
#include "Adafruit_ILI9340.h"
#define _cs 10
#define _dc 9
#define _rst 8

Adafruit_ILI9340 TFTscreen = Adafruit_ILI9340(_cs, _dc, _rst);
//sampling frequency
const int freq =80;  
double freqd = freq;
const int WinPeriod = 4; //in sec
//analog read at A0
int InPin = 0;
int RedIn = 0;
//digital output pin
const int Redpin = 7 ;   //Arduino outpout pin to control BJT switch on or off
const int IRpin = 6;     //Arduino outpout pin to control BJT switch on or off
//change the magnitude on display screen
const double ampR = 2.0;     // enlarge or decrease the Red waveform on the screen
const int posRY = -250;      //  modify the Red waveform y position on the screen
const double ampIR = 0.5;    // enlarge or decrease the IR waveform on the screen
const int posIRY = 80;       //  modify the IR waveform y position on the screen
//finding maximum
double findmaxR[3]={0,0,0};     //find max of Red waveform 
double findmaxIR[3]={0,0,0};    //find max of IR　waveform
//save maximum values and indexes
const int pixel_len = 320;      //length of whole display screen
const int pix = 30;             //size of the threshold array
double thres_arrR[pix];         // save value of Red waveform from 
int thres_indR[pix];            //save index of Red maximum waveform from     range:0-319
double thres_arrIR[pix];         // save value of Red waveform from 
int thres_indIR[pix];            //save index of Red maximum waveform from     range:0-319
int thres_save[pix];             //save index of Red maximum after filtering like interval > 27  && < 130 
int thres_indexR=0;              //used for the index number of thres_arrR and thres_indR arrays   ex: thres_arrR[thres_indexR] 
int thres_indexIR = 0;
//digital filter's design
const int order = 3;
int prevRedIn[2 * order + 1];// = {0, 0, 0, 0, 0};
int IRIn = 0;
int prevIRIn[2 * order + 1]; //= {0, 0, 0, 0, 0};
//display
double RedAC[pixel_len];       
double IRAC[pixel_len ];
double RedDC[pixel_len];
double IRDC[pixel_len ];
double Red_wave[pixel_len];     //the array used for display on the screen
double IR_wave[pixel_len ];     //the array used for display on the screen
//down sampling if used
const int sam_pixel = 1;         //sampling frequency, like average three value then write one pixel on the screen, but is not used in this version
double Red_buf[sam_pixel]={0};//,0,0,0,0};
double IR_buf[sam_pixel]={0};//,0,0,0,0};
double Redsam_wave[pixel_len];
double IRsam_wave[pixel_len];
//count for collecting data loop
int Size = 0;
int count=0;
double loopPer = 1 / freqd * 1000000; //in us
//filters
//bandpass with range 0.5-3.8Hz(30-220bpm), 3rd order for 80Hz
double RACb[7] = {0.001706723119848,0,-0.005120169359544,0,0.005120169359544,0,-0.001706723119848};
double RACa[6] = {-5.450738550842415,12.422272612518626,-15.155363550277329,10.441537955263303,-3.852323448583826,0.594616240753234};
//lowpass cutoff @0.5Hz, 2nd order
double RDCb[3] = {0.00024136, 0.00048272, 0.00024136};
double RDCa[2] = { -1.9556, 0.9565};
//extremum's indexes and values
int maxR_i = 0;    //save the index (x position) of Red maximum 
int maxIR_i =0;    //save the index (x position) of IR maximum 
int minR_i =0;     //save the index (x position) of Red minimum
int minIR_i = 0;   //save the index (x position) of IR maximum 
double sumIR = 0;   //sum of IR
double sumR = 0;    //sum of Red
double dissumR = 0;  //the display of sum on the screen
double maxR=0;   //save the value (y position) of Red maximum 
double maxIR=0;   //save the value (y position) of IR maximum 
double minR=0;    //save the value (y position) of Red minimum  
double minIR = 0;    //save the value (y position) of IR minimum  
double disthresR[pixel_len];    //
//double thres_save = 0;
//double disminR = 0;
//double dismaxR = 0;

//display
int dcount = 0;   //used for determination whether doing calculation or display HR or SpO2
String strPulse;
String R_string ;
double PulseR = 0;
double SpO2[4];
double RR = 0;    // = R[0]

void setup() {
  Serial.begin(250000);
  analogWriteResolution(12);
  analogReadResolution(12);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  pinMode(Redpin,OUTPUT);
  pinMode(IRpin,OUTPUT);
  digitalWrite(Redpin,LOW);
  digitalWrite(IRpin,LOW);
  for(int i = 0 ;i < 2 * order + 1 ; i++){ //2*order+1
    prevRedIn[i] = 0 ;
    prevIRIn[i] = 0 ;
    RedAC[i] = 0;
    IRAC[i] = 0;
    RedDC[i] = 0;
    IRDC[i] = 0;
  }
//initialize display
  TFTscreen.begin();
  TFTscreen.setRotation(3);
  TFTscreen.fillScreen(ILI9340_BLACK);
  TFTscreen.setCursor(0,0);
  TFTscreen.setTextSize(1);
  TFTscreen.println("AlphaHKU");
  TFTscreen.setTextSize(2);
  TFTscreen.println("HR  :\nSPO2 :");
  TFTscreen.setTextSize(1);  

  for(int i = 0 ; i < pixel_len ; i++){  //initialize the display-related arrays
     Red_wave[i]=240;
     IR_wave[i]=240;
     Redsam_wave[i]=240;
     IRsam_wave[i]=240;
      disthresR[i]=0;
}
  digitalWrite(Redpin,HIGH);
  delayMicroseconds(1000);
  for(int i=0;i<20;i++){    //initialize the threshold-related arrays
     thres_indR[i] = 0;
     thres_arrR[i] = 0;
     thres_indIR[i] = 0;
     thres_arrIR[i] = 0;
     thres_save[i] = 0;
  }
}

void loop() {
  int loopstart = micros();
  //Serial.println("loopstart");
  int PeriodSize = freq * WinPeriod;  //320
  
  if (Size < PeriodSize) {  //PerioaSize = 80 * 4 = 320 
    int start = micros(); //time :　0
   
    for (int ii = 0; ii < 2*order; ii++) { //2 * order 
      prevRedIn[2*order - ii] = prevRedIn[2*order - ii - 1];  //2 * order 
      prevIRIn[2*order - ii] = prevIRIn[2*order - ii - 1]; //2 * order 
    }
    Size++;
    prevRedIn[0] = RedIn;      //RedIn =0   initial
    RedAC[Size - 1] = filters(RACb, RACa, RedIn, prevRedIn, RedAC, 2*order, Size);
    RedDC[Size - 1] = filters(RDCb, RDCa, RedIn, prevRedIn, RedDC, 2, Size);
  
     //find Red maximum
      for(int i = 0 ; i < 2 ; i++){
        findmaxR[i] = findmaxR[i+1];
      }
       findmaxR[2] = RedAC[Size-1];
       if(findmaxR[1] < findmaxR[0] and findmaxR[1] < findmaxR[2]){
        thres_arrR[thres_indexR]=findmaxR[1];
        thres_indR[thres_indexR] = Size -1 ;
        thres_indexR++;
       }
    
    //sum of RedAC array
    sumR += RedAC[Size-1];
    dissumR += ((-RedAC[Size-1]*100/10240+1.5)*TFTscreen.height()/2)*ampR+posRY;
 
    if(maxR < RedAC[Size-1]){
      maxR = RedAC[Size-1];
      maxR_i = Size  - 1 ;
    }
    if(minR > RedAC[Size-1]){
      minR = RedAC[Size-1];
      minR_i = Size - 1;
    }
    //Serial.println(maxR);
    //Serial.println(minR);
    
    //size 0,1,2,3..319
    TFTscreen.drawRect((Size)%TFTscreen.width(),40,1,240,ILI9340_WHITE); // set the screen to white.

    unsigned long middle = micros();
    //Serial.println(middle - start);  //time :　2300    -> RedIn don't need delay
    
    RedIn = analogRead(InPin);
    digitalWrite(Redpin,LOW);
    
    prevIRIn[0] = IRIn;
    digitalWrite(IRpin,HIGH);
    
    delayMicroseconds(1000);
    IRIn = analogRead(InPin);
    digitalWrite(IRpin,LOW);
    digitalWrite(Redpin,HIGH);
   
    IRAC[Size - 1] = filters(RACb, RACa, IRIn, prevIRIn, IRAC, 2*order, Size);
    IRDC[Size - 1] = filters(RDCb, RDCa, IRIn, prevIRIn, IRDC, 2, Size);

    //find IR maximum
      for(int i = 0 ; i < 2 ; i++){
        findmaxIR[i] = findmaxIR[i+1];
      }
       findmaxIR[2] = IRAC[Size-1];
       if(findmaxIR[1] <  findmaxIR[0] and findmaxIR[1] < findmaxIR[2]){
        thres_arrIR[thres_indexIR]=findmaxIR[1];
        thres_indIR[thres_indexIR] = Size -1 ;
        thres_indexIR++;
       }

    if(maxIR < IRAC[Size-1]){
      maxIR = IRAC[Size-1];
      maxIR_i = Size  - 1 ;
    }
    if(minIR > IRAC[Size-1]){
      minIR = IRAC[Size-1];
      minIR_i = Size - 1;
    }
    
    double OutRed=RedAC[Size -1]*100;
    double OutIR=IRAC[Size -1]*1000;
    
    int posR = int((-OutRed/10240+1.5)*TFTscreen.height()/2);
    if( posR < 40 ){
      posR = 40;
    }
    
    int posIR = int((-OutRed/10240+1.5)*TFTscreen.height()/2);
    if( posIR < 40 ){
      posIR = 40;
    }
   unsigned long last = micros();   
  } 
else{
    unsigned long A = micros();
    //scanning
    
   // if index = 0, means the  extremum is disappear and find a new one
    if(maxR_i < 0){
      maxR = 0;
      sumR = 0;
      dissumR = 0;
      for(int i=0;i<320;i++){
        if(maxR < RedAC[i]){    // if wasn't add in the IP 14 version, so maxR_i will always become 319, perhaps HR error is caused by this
        maxR = max(maxR,RedAC[i]);
        maxR_i = i;
        }
        sumR = sumR + RedAC[i];
        dissumR += ((-RedAC[i]*100/10240+1.5)*TFTscreen.height()/2)*ampR+posRY;
      }
    }
    if(minR_i < 0){
      minR = 0;
      sumR = 0;
      dissumR = 0;
      for(int i=0;i<320;i++){
        if(minR > RedAC[i]){    // if wasn't add in the IP 14 version, so minR_i will always become 319, perhaps HR error is caused by this
        minR = min(minR,RedAC[i]);
        minR_i = i;
        }
        sumR = sumR + RedAC[i];
        dissumR += ((-RedAC[i]*100/10240+1.5)*TFTscreen.height()/2)*ampR+posRY;
      }
    }
    if(maxIR_i < 0){
      maxIR = 0;
      sumIR = 0;
      for(int i=0;i<320;i++){
        if( maxIR < IRAC[i]){
        maxIR = max(maxIR,IRAC[i]);
        maxIR_i = i;
        }
      }
    }
    if(minIR_i < 0){
      minIR = 0;
      sumIR = 0;
      for(int i=0;i<320;i++){
        if(minIR > IRAC[i]){
        minIR = min(minIR,IRAC[i]);
        minIR_i = i;
        }
      }
    }
    //Red sample   in first time the time is not enough
    RedIn = analogRead(InPin);
    //off Red switch
    digitalWrite(Redpin,LOW);
    //on IR switch
    unsigned long B = micros();
    //switching the arrays and Red filter and write red pixel and erase two old data
    digitalWrite(IRpin,HIGH);
 
    //left shift the arrays, also the indexes
    for (int ii = 1; ii < Size; ii++) {
      RedAC[ii - 1] = RedAC[ii];
      IRAC[ii - 1] = IRAC[ii];
      RedDC[ii - 1] = RedDC[ii];
      IRDC[ii - 1] = IRDC[ii];
    }
      
    maxR_i--;
    maxIR_i--;
    minR_i--;
    minIR_i--;
    
    for (int i = 1 ; i < pix ; i++){
      thres_indR[i-1]-- ;               // [20,30,40,50]  ->  [19,29,39,49]
      thres_indIR[i-1]-- ;
    }
    //if threshold index is out of array, then left shift the thres_arr
    if(thres_indR[0] == -1){
      for(int i = 1;i < pix ;i++){
        thres_indR[i-1] = thres_indR[i];    // [-1,20,40,60,80]  ->  [20,40,60,80,80]
        thres_arrR[i-1] = thres_arrR[i];
      }
       thres_indexR--;   // the index of thres_indR and thres_arrR arrays
    }
    if(thres_indIR[0] == -1){
      for(int i = 1;i < pix ;i++){
        thres_indIR[i-1] = thres_indIR[i];
        thres_arrIR[i-1] = thres_arrIR[i];
      }
      thres_indexIR--;
    }
      for (int ii = 0; ii < 2*order; ii++) {
      prevRedIn[ 2 * order - ii] = prevRedIn[ 2 * order - ii - 1];
      prevIRIn[ 2 * order - ii] = prevIRIn[ 2 * order - ii - 1];
    }
  
    count++;
    count %= TFTscreen.width() * sam_pixel;  //320   count:0~320   *3 0~960 
 
    //filter
    prevRedIn[0] = RedIn;
    RedAC[PeriodSize - 1] = filters(RACb, RACa, RedIn, prevRedIn, RedAC, 2*order, Size);
    RedDC[PeriodSize - 1] = filters(RDCb, RDCa, RedIn, prevRedIn, RedDC, 2, Size);

     //find Red maximum
      for(int i = 0 ; i < 2 ; i++){
        findmaxR[i] = findmaxR[i+1];
      }
       findmaxR[2] = RedAC[Size-1];
       if(findmaxR[1] < findmaxR[0] and findmaxR[1] < findmaxR[2]){
        thres_arrR[thres_indexR] = findmaxR[1];
        thres_indR[thres_indexR] = Size -1 ;   
        thres_indexR++;
       }

    if(maxR < RedAC[PeriodSize-1]){
      maxR = RedAC[PeriodSize-1];
      maxR_i = PeriodSize  - 1 ;
    }
    if(minR > RedAC[PeriodSize-1]){
      minR = RedAC[PeriodSize-1];
      minR_i = PeriodSize - 1;
    }
     
    //draw screen white
    TFTscreen.writePixel(count%TFTscreen.width(), Red_wave[count%TFTscreen.width()]   ,ILI9340_WHITE);
    TFTscreen.writePixel(count%TFTscreen.width(), IR_wave[count%TFTscreen.width()] ,ILI9340_WHITE);

    double OutRed=RedAC[PeriodSize -1]*100;
    int posR = int((-OutRed/10240+1.5)*TFTscreen.height()/2);
    posR = posR * ampR + posRY;
    if(posR < 40){
      posR = 40;
    }

    //used for sample rate is not one
    Red_buf[(count%sam_pixel)] = posR;
    Red_wave[count%TFTscreen.width()] = posR; 
    TFTscreen.writePixel(count%TFTscreen.width(), Red_wave[count%TFTscreen.width()] ,ILI9340_RED);
    
    unsigned long IR_end = micros();
    //IR sample
    IRIn = analogRead(InPin);
    //off IR switch
    digitalWrite(IRpin,LOW);
    //on Red switch
    digitalWrite(Redpin,HIGH);
    unsigned long C = micros();
    //IR fiter
    prevIRIn[0] = IRIn;
    IRAC[PeriodSize - 1] = filters(RACb, RACa, IRIn, prevIRIn, IRAC, 2 * order, Size);
    IRDC[PeriodSize - 1] = filters(RDCb, RDCa, IRIn, prevIRIn, IRDC, 2, Size);

     //find IR maximum
      for(int i = 0 ; i < 2 ; i++){
        findmaxIR[i] = findmaxIR[i+1];
      }
       findmaxIR[2] = IRAC[Size-1];
       if(findmaxIR[1] < findmaxIR[0] and findmaxIR[1] < findmaxIR[2]){
        thres_arrIR[thres_indexIR] = findmaxIR[1];
        thres_indIR[thres_indexIR] = Size -1 ;
        thres_indexIR++;
      }

    if(maxIR < IRAC[PeriodSize-1]){
      maxIR = IRAC[PeriodSize-1];
      maxIR_i = PeriodSize  - 1 ;
    }
    if(minIR > IRAC[PeriodSize-1]){
      minIR = IRAC[PeriodSize-1];
      minIR_i = PeriodSize - 1;
    }
    
    double OutIR=IRAC[PeriodSize -1]*1000;
    int posIR=int((-OutIR/10240+1.5)*TFTscreen.height()/2);
    
    posIR = posIR * ampIR + posIRY;
    if(posIR < 40){
      posIR = 40;
    }    
    
    //used for sampling rate is not one
    IR_buf[count%sam_pixel] = posIR;
    IR_wave[count%TFTscreen.width()] = posIR;

    TFTscreen.writePixel(count%TFTscreen.width(), IR_wave[count%TFTscreen.width()] ,ILI9340_BLUE);

   //    if(count % sam_pixel == 0 ){  //count = 3 6 9 
//     //  Serial.println("count %5 ==0");
//    
//      TFTscreen.writePixel(count/sam_pixel,Redsam_wave[count/sam_pixel] +50,ILI9340_WHITE);
//      //TFTscreen.writePixel(count/sam_pixel,IRsam_wave[count/sam_pixel]  +50,ILI9341_WHITE);
//      double buf_R=0;
//      double buf_IR=0;
//      for(int i=0;i< sam_pixel;i++){
//        buf_R += Red_buf[i];
//        //buf_IR += IR_buf[i];
//        //Serial.print("Red_buf: ");
//        //Serial.println(Red_buf[i]);
//        //Serial.print("IR_buf: ");
//        //Serial.println(IR_buf[i]);
//      }
//      Redsam_wave[count/sam_pixel] = buf_R/sam_pixel;
//      IRsam_wave[count/sam_pixel]=buf_IR/sam_pixel;
//      //TFTscreen.writePixel(count/sam_pixel,Redsam_wave[count/sam_pixel]+50,ILI9341_BLACK);
//      //TFTscreen.writePixel(count/sam_pixel,IRsam_wave[count/sam_pixel]+50,ILI9341_YELLOW);
//    }  

//===============================calculation===========================
   
  if(dcount % 10 != 1 ){   //in ten loops, calculate nine times and display once  
      //   1~9 loops
    unsigned long D = micros();
    //Threshold finding
    double maxR_RDC = 0;
    double maxR_IR = 0;
    double maxR_IRDC = 0;
    double minR_RDC = 0;
    double minR_IR = 0;
    double minR_IRDC = 0;
    double maxIR_RDC = 0;
    double maxIR_R = 0;
    double maxIR_IRDC = 0;
    double minIR_RDC = 0;
    double minIR_R = 0;
    double minIR_IRDC = 0;

    maxR_RDC = RedDC[maxR_i];
    maxR_IR = IRAC[maxR_i];
    maxR_IRDC = IRDC[maxR_i];
  
    minR_RDC = RedDC[minR_i];
    minR_IR = IRAC[minR_i];
    minR_IRDC = IRDC[minR_i];
         
    maxIR_RDC = RedDC[maxIR_i];
    maxIR_R = RedAC[maxIR_i];
    maxIR_IRDC = IRDC[maxIR_i];
 
    minIR_RDC = RedDC[minIR_i];
    minIR_R = RedAC[minIR_i];
    minIR_IRDC = IRDC[minIR_i];

    unsigned long E = micros();
    //threshold setting and pulse finding 
    double meanR = sumR / (PeriodSize);
    double thresR = minR + (meanR - minR) / 2;  // maxR-(maxR-minR)/6;
    double PulPer[10];
    
    for (int i=0;i<10;i++){
      PulPer[i] = 0 ;    //initialize PulPer array
    }
    
    TFTscreen.writePixel(count%TFTscreen.width()  , disthresR[count%TFTscreen.width()] ,ILI9340_WHITE);
   
    disthresR[count%TFTscreen.width()] =  ((-thresR*100/10240+1.5)*TFTscreen.height()/2) * ampR + posRY; //disminR + (dissumR/PeriodSize - disminR) / 2;
   
    double sumPer = 0;
    double meanPer = 0;
    int aftlen_thresPos=0;

    TFTscreen.writePixel(count%TFTscreen.width(), disthresR[count%TFTscreen.width()] ,ILI9340_BLACK);
    
    int index = 1 ; // the 0 index is used for last loop's last index
    
    for (int ii = 0 ; ii < thres_indexR - 1 ; ii++) {
    if(thres_arrR[ii] < thresR ){   //if the value in arrR is smaller than threshold, save it.
      thres_save[index] = thres_indR[ii];
       index++;
      }
    }
    
    Serial.println(thresR);
    
    Serial.println("thres_save");
    for(int i=0;i<10;i++){
      Serial.println(thres_save[i]);
    }
    
    Serial.println();
    
    for(int ii=0; ii< thres_indexR - 1;ii++){
      if(thres_save[ii + 1 ] - thres_save[ii] > 27 && thres_save[ii+1] - thres_save[ii] < 130){  //3Hz
            aftlen_thresPos++;
            sumPer += (thres_save[ii+1]-thres_save[ii]) / freqd ;    //method one
            PulPer[ii] = (thres_save[ii+1]-thres_save[ii])/freqd;    //method two
            Serial.println(PulPer[ii]);
            Serial.println(sumPer);
      }
    }
 
    Serial.println(aftlen_thresPos);

    //need to do the array sorting here 
    // this version the PulPer array is not ordered
    
    if(aftlen_thresPos - 1 != 0 and sumPer != 0){
      
      // method two
      if(aftlen_thresPos % 2 ==0){
        meanPer = (PulPer[aftlen_thresPos/2-1]+PulPer[aftlen_thresPos/2])/2;   //ex:aftlen_thresPos = 4   mean is 2 and 3 
      }
      else {
        meanPer = ( PulPer[aftlen_thresPos/2] );
      }
      //method one
      //meanPer = sumPer / (aftlen_thresPos-1);  
        PulseR = 1 / meanPer * 60;
         Serial.println(PulseR);
     }
   
      if(thres_save[1] == 0 ){  // if the first index (not the zeroth index) of thres_save is 0, then it become the zeroth index 
      thres_save[0] = thres_save[1];
    }
    else if (thres_save[0] < -20){
      thres_save[0] = thres_save[1];  //if the zeroth index is too small, replace it with first index
    }
    else if (thres_save[index-1] > 300){
      thres_save[0] = thres_save[index-1] - 319 ; //save the last loop's last index
    }
    
    for(int i = 1 ; i < pix ; i++){   //initialize the thres_save array
      thres_save[i] = 0 ;   //only keep the zeroth index of the array, other value will do it in the new loop
    }
    
    unsigned long F = micros();
    //calculate R
    double R[4];
    R[0] = log((maxR + maxR_RDC) / maxR_RDC) / log((maxR_IR + maxR_IRDC) / maxR_IRDC);
    R[1] = log((maxIR_R + maxIR_RDC) / maxIR_RDC) / log((maxIR + maxIR_IRDC) / maxIR_IRDC);
    R[2] = log((minR + minR_RDC) / minR_RDC) / log((minR_IR + minR_IRDC) / minR_IRDC);
    R[3] = log((minIR_R + minIR_RDC) / minIR_RDC) / log((minIR + minIR_IRDC) / minIR_IRDC);
    RR = R[0];
    //calculate SpO2
    double AA = 1;
    double BB = 1;
    double CC = 1;
    double DD = 1;

    for (int ii = 0; ii < 4; ii++) {
      SpO2[ii] = (AA - BB * R[ii]) / (CC - DD * R[ii]);
    }
    
    unsigned long Red_end = micros();  
    //Serial.print("Red time: ");
    //Serial.println(Red_end-Red_start);
    dcount ++;

}// end of else if dcount%10 != 0         

//===============================display HR=============================

else if (dcount %20 == 1){  // display HR on screen   only  21 41 61 
      
    unsigned long G = micros();
    TFTscreen.fillRect(80,10,40,30,ILI9340_GREEN);   //in old version there is a rect black block, I change it to green to see if it is caused by this line 
    strPulse=String(PulseR);
   
    char disPulse[3];    // one tft write take 1500 microseconds
    strPulse.toCharArray(disPulse,3);
    TFTscreen.setTextColor(ILI9340_WHITE);
    TFTscreen.setCursor(80,12);
    TFTscreen.println(strPulse);
    dcount ++;
}//end of else if dcount % 10 == 0

//=======================display SpO2========================

else if(dcount % 10 ==1 && dcount % 20 != 1){   //display SpO2 on screen  //11 31 51   SpO2
 
  unsigned long H = micros();
   TFTscreen.fillRect(50,30,30,20,ILI9340_BLACK);
    R_string=String(RR);
    
    char R_dis[3];    // one tft write take 1500 microseconds
    R_string.toCharArray(R_dis,3);
    TFTscreen.setTextSize(1.6);
    TFTscreen.setCursor(80,30);
    TFTscreen.println(R_string);
    dcount ++;    
  } //end display SpO2
}   //end else

  //End loop time control
  int loopend = micros();
  int delaymic = loopPer - (loopend - loopstart);
    Serial.print("delaymic: ");
    Serial.println(delaymic);
    if(delaymic >0){
    delayMicroseconds(delaymic);
    }
}

double filters(double b[], double a[], int x, int prevx[], double prevy[], int order, int Size) {
  if (Size - 1 < order) {
    double y = b[0] * double(x);
    for (int ii = 0; ii < Size - 1; ii++) {
      y = y + b[ii + 1] * double(prevx[ii+1]) - a[ii] * prevy[Size - 2 - ii];
    }
    //cout<<y;
    return y;
  } else {
    double y = b[0] * double(x);
    for (int ii = 0; ii < order; ii++) {
      y = y + b[ii + 1] * double(prevx[ii+1]) - a[ii] * prevy[Size - 2 - ii];
    }
    return y;
  }

}

