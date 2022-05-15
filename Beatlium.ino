/*
 * 2022/05/08(Sun)
 * Developper:Kabosuxu
 * Opensource
 */

#include <arduinoFFT.h>                                  
#define MIC 35                                                              // InputDevice(MAX4466) is conneccted to 35 Pin.
/* Name  Frequency      Color   Array  PinNo(ESP32)
 *  
 *  
 */
#define fftsamples 2048
#define SAMPLING_FREQUENCY 10000
double vReal[fftsamples];
double vImag[fftsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, fftsamples, SAMPLING_FREQUENCY);

#include <Adafruit_NeoPixel.h>
const int PIN[6] = {14, 27, 26, 25, 33, 32};                               // OutputDevice(SK6812) is conneccted to 14,27,26,25,33,32 Pin.
/* Name  Frequency      Color   Array  PinNo(ESP32)
 *  LOW   1    ~ 100  Hz Purple  PIN[0] 14
 *  MID1  101  ~ 250  Hz Pink    PIN[1] 27
 *  MID2  251  ~ 500  Hz Cyan    PIN[2] 26
 *  MID3  501  ~ 1000 Hz Red     PIN[3] 25
 *  HIGH1 1001 ~ 1500 Hz Orange  PIN[4] 33
 *  HIGH2 1501 ~ 2500 Hz Yellow  PIN[5] 32
*/
#define NUMPIXELS 1
int color[6][3] = {{255, 0, 255},                                          // Purple
                   {255, 123, 123},                                        // Pink 
                   {0, 255, 255},                                          // Cyan
                   {255, 0, 0},                                            // Red
                   {255, 123, 0},                                          // Orange
                   {255, 255, 0}};                                         // Yellow

Adafruit_NeoPixel pixels_LOW(NUMPIXELS, PIN[0], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_MID1(NUMPIXELS, PIN[1], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_MID2(NUMPIXELS, PIN[2], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_MID3(NUMPIXELS, PIN[3], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_HIGH1(NUMPIXELS, PIN[4], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_HIGH2(NUMPIXELS, PIN[5], NEO_GRB + NEO_KHZ800);

double avg[6] ={0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0};                         // SoundPower average datata

void setup() {
  // 1:Setting
  Serial.begin(115200);                                                     // Begin ing baurate for serial monitor
  pinMode(MIC, INPUT);                                                      // Set MAX4466 Pin 
  allpixels_begin();                                                        // Userfunction 1
  allpixels_clear();                                                        // Userfunction 2
  // 2:Test Light
  for (int i = 0; i < NUMPIXELS; i++) {
    allpixels_clear();                                                      
    allpixels_setbright();
    allpixels_show();
    delay(100);
  }
}

void loop() {
  sample(fftsamples);                               //01:Sample audio data           [Userfunction01]
  DCRemoval2(vReal, fftsamples);                    //02:Remove DC from samplingdata.[Userfunction02]
  //Serial.println("Sampling Data");
  //drawChart_Smp(fftsamples);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  //03:Set FFTwindow
  FFT.Compute(FFT_FORWARD);                         //04:FFT
  FFT.ComplexToMagnitude();                         //05:Translate Comp to Real
  calc_power(fftsamples / 2);                       //06:Calculate spectrum power    [Userfunction03]
  lightup();                                        //07:Lightup 6 herbariums
}

/*
 * Userfunction01"allpixels_begin"
 * overview    : begin 6 RGB LEDs
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void allpixels_begin() {
  pixels_LOW.begin();
  pixels_MID1.begin();
  pixels_MID2.begin();
  pixels_MID3.begin();
  pixels_HIGH1.begin();
  pixels_HIGH2.begin();
}

/*
 * Userfunction02"llpixels_setbright"
 * overview    : set the default bright(123) to 6 RGB LEDs.
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void allpixels_setbright() {
  pixels_LOW.setBrightness(123);
  pixels_MID1.setBrightness(123);
  pixels_MID2.setBrightness(123);
  pixels_MID3.setBrightness(123);
  pixels_HIGH1.setBrightness(123);
  pixels_HIGH2.setBrightness(123);
}

/*
 * Userfunction03"allpixels_clear"
 * overview    : clear the color information of 6 RGB LEDs.
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void allpixels_clear() {
  pixels_LOW.clear();
  pixels_MID1.clear();
  pixels_MID2.clear();
  pixels_MID3.clear();
  pixels_HIGH1.clear();
  pixels_HIGH2.clear();
}

/*
 * Userfunction04"allpixels_setcolor"
 * overview    : set the color information of 6 RGB LEDs.
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void allpixels_setcolor() {
  pixels_LOW.setPixelColor(0, pixels_LOW.Color(color[0][0], color[0][1], color[0][2]));
  pixels_MID1.setPixelColor(0, pixels_MID1.Color(color[1][0], color[1][1], color[1][2]));
  pixels_MID2.setPixelColor(0, pixels_MID2.Color(color[2][0], color[2][1], color[2][2]));
  pixels_MID3.setPixelColor(0, pixels_MID3.Color(color[3][0], color[3][1], color[3][2]));
  pixels_HIGH1.setPixelColor(0, pixels_HIGH1.Color(color[4][0], color[4][1], color[4][2]));
  pixels_HIGH2.setPixelColor(0, pixels_HIGH2.Color(color[5][0], color[5][1], color[5][2]));
}

/*
 * Userfunction05"allpixels_show"
 * overview    : show 6 RGB LEDs.
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void allpixels_show() {
  pixels_LOW.show();
  pixels_MID1.show();
  pixels_MID2.show();
  pixels_MID3.show();
  pixels_HIGH1.show();
  pixels_HIGH2.show();
}

/*
 * Userfunction06 "sample" 
 * overview    : Sample the audio data.
 * returnvalue : none
 * argument    : (int)nsamples samplingdata length 
 * Made by Made by Takehiko Shimojima.
 * https://gist.github.com/TakehikoShimojima/13782a144548d1d77fa5e2ff1bc57411#file-sound_fft-ino
 */
void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    unsigned int sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    unsigned long t = micros();
    //vReal[i] = analogRead(MIC)/4095.0 *3.6 + 0.1132;
    vReal[i] = analogRead(MIC);
    vImag[i] = 0;
    while ((micros() - t) < sampling_period_us) ;
  }
}
/*
 * Userfunction07 "DCRemoval2" 
 * overview    : Removal the DC from sampling data.
 * returnvalue : none
 * argument    : (double)*vData
 *               (uint16_t)samples
 * Made by Takehiko Shimojima.
 * https://gist.github.com/TakehikoShimojima/13782a144548d1d77fa5e2ff1bc57411#file-sound_fft-ino
 */
void DCRemoval2(double *vData, uint16_t samples) {
  double mean = 0;
  // calculate mean
  for (uint16_t i = 1; i < samples; i++) {
    mean += vData[i];
  }
  mean /= samples;
  for (uint16_t i = 1; i < samples; i++) {
        vData[i] -= mean;
  }
}

/*
 * Userfunction08 "calc_power" 
 * overview    : Calculate the spectrum power.
 * returnvalue : none
 * argument    : (int)nsamples samplingdata length 
 * Made by Kabosuxu
 */
void calc_power(int nsamples) {
    double sum[6] ={0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0}; 
    int avg_div[6] ={21, 31 , 51 , 102 , 103 , 102};
    //Serial.println("sound level !!");
    for (int band = 0; band < nsamples; band++) {
      int df = ( band * SAMPLING_FREQUENCY ) / fftsamples;
      double d = vReal[band]/nsamples;
      if(d > 30){
        if(band <= 20){
          sum[0] += d;
        }else if((21 <= band) && (band <= 51)){
          sum[1] += d;
        }else if((52 <= band) && (band <= 102)){
          sum[2] += d;
        }else if((103 <= band) && (band <= 204)){
          sum[3] += d;
        }else if((205 <= band) && (band <= 307)){
          sum[4] += d;
        }else if((308 <= band) && (band <= 409)){
          sum[5] += d;
        }
      }
    }
    for(int i = 0; i < 6; i++){
      //Serial.print("Freq[");
      Serial.print(i);
      //Serial.print(":");
      avg[i] = sum[i] / avg_div[i];
      Serial.println(avg[i]);
      //Serial.print(",");
    }
    //Serial.println("End");
}


/*
 * Userfunction09"lightup"
 * overview    : light up 6 RGB LEDs according to the frequency.
 * returnvalue : none
 * argument    : none
 * Made by Kabosuxu
 */
void lightup(){
  allpixels_clear();                                            // Userfunction 3
  int adj = 10;
  for (int i = 0; i < NUMPIXELS; i++) {
    allpixels_clear();                                          // Userfunction 3
    allpixels_setcolor();                                       // Userfunction 4
    pixels_LOW.setBrightness(avg[0]*adj);
    pixels_MID1.setBrightness(avg[1]*adj);
    pixels_MID2.setBrightness(avg[2]*adj);
    pixels_MID3.setBrightness(avg[3]*adj);
    pixels_HIGH1.setBrightness(avg[4]*adj);
    pixels_HIGH2.setBrightness(avg[5]*adj);
    allpixels_show();                                           // Userfunction 5
  }
}
