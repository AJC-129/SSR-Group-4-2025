/*
 * Starter code for a sound-following robot that uses
 * two microphones to drive toward a sound source.
 *
 * Originally written for Arduino UNO.
 *
 * Z.Francis-Cox@soton.ac.uk
 * 
 */

// declare constant variables for motor control pins
const int leftMotorForwardPin = 4;
const int leftMotorBackwardPin = 5;
const int rightMotorForwardPin = 2;
const int rightMotorBackwardPin = 3;
const int leftMotorSpeedPin = 9;
const int rightMotorSpeedPin = 10;

// constant variables for microphone pins
const int mic1pin = A1;
const int mic2pin = A2;

// other variables
const int sample_window = 500; // amount of time to sample sound in milliseconds
int mic1;       // microphone 1 value from analogRead (0-255)
int mic2;       // microphone 2 value from analogRead (0-255)

float gain1 = 1.0;  // Microphone 1 gain
float gain2 = 1.0;  // Microphone 2 gain


// Physical constants
const float c = 344.0;        // speed of sound (m/s)
const float d12 = 0.086;      // distance between microphones 1 and 2 (m)

const double pi = 3.141592654;

// Timing
double tsm;                       // measured sampling delta time (s)
unsigned long start_time;         // time in milliseconds at start of sample window (since program started)
unsigned long current_time;       // current time in milliseconds (since program started)
unsigned long sample_time_start;  // Time at start of N samples
unsigned long sample_time_end;    // Time at end of N samples over M channels


// Cross Correlation calculation variables
const int N = 200;        // Number of samples per cc calculation
float oneByN = 1.0 / N;
int count;                // Number of loops to calculate mean of cc values over time

int8_t sample1[N];        // Input buffer array - microphone 1
int8_t sample2[N];        // Input buffer array - microphone 2

float cc_mean12;        // Mean of maximum cc index (lag) values Mic 1 to Mic 2
int cc_lag12;           // Maximum lag value Mic 1 to Mic 2
int cc_lag12_sum;       // Sum of maximum index (sample lag) values Mic 1 to Mic 2

// Angle calculation variables
double angle_deg121;
double angle_deg122;

void setup() {
  // initialise ADC to increase max sampling rate to approx. 62.5 kHz
  ADCSRA = 0b10000100;  // set prescaler to 16 to increase sampling rate
  ADCSRB = 0b00000000;  // enable ADC free running mode
  ADMUX |= (1 << ADLAR);  // Left align ADC value (only read highest 8 bits)

  Serial.begin(921600);  // initialize serial communication for debugging
  delay(1000);

  // set motor control pins as outputs
  pinMode(leftMotorForwardPin,OUTPUT);
  pinMode(leftMotorBackwardPin,OUTPUT);
  pinMode(rightMotorForwardPin,OUTPUT);
  pinMode(rightMotorBackwardPin,OUTPUT);
  pinMode(leftMotorSpeedPin,OUTPUT);
  pinMode(rightMotorSpeedPin,OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(leftMotorSpeedPin,HIGH);
  digitalWrite(rightMotorSpeedPin,HIGH);
}


void loop() {
  // main code loop to run repeatedly:==============================================

  stopDriving();  // Stop motors
  delay(500);

  count = 0;    // Set/reset averaging counter

  // Initialise time values for current loop
  start_time = millis();
  current_time = millis();

  cc_lag12_sum = 0;


  // Read microphone data and calculate average sample delay
  while( current_time - start_time < sample_window){
    
    // Read microphone inputs and save them to arrays of length N 
    // (DO NOT add to this for-loop as it will increase the sampling time)
    sample_time_start = micros();
    for (int i = 0; i < N; i++){
      sample1[i] = analogRead(mic1pin);
      sample2[i] = analogRead(mic2pin);
    }

    sample_time_end = micros();

    // Calculate actual sampling time
    tsm = (sample_time_end - sample_time_start) * oneByN;  // divide by N samples
    tsm = tsm / 1000000;                                   // convert to s from micros

    //Serial.println(tsm, 6);

    // normalise sampled data (remove if using Magnitude Difference)
    normalise(sample1);
    normalise(sample2);

    // print microphone sample values to console
    for (int i = 0; i < N; i++){
      Serial.print(sample1[i]);
      Serial.print(" ");
      Serial.println(sample2[i]);
    }

    // Calculate sample delay (lag) between channels using cc (see function below)
    cc_lag12 = cross_correlation_lag(sample1, sample2);

    cc_lag12_sum += cc_lag12;   // add max index number (sample lag) to sum for averaging
    
    count += 1;                 // increment counter for averaging
    current_time = millis();    // time at end of averaging
  }

  //Serial.println(count);
  // Calculate mean of maximum index (lag) values over sample window to get stable delay time
  cc_mean12 = cc_lag12_sum / count;
  //Serial.println(cc_mean12);



  /* Insert robot control algorithm here ==============================================
  *
  *
  * e.g., if angle/delay is within a certain range, do something with the motors.
  *
  * Motor movement functions defined at end of file.
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *
  *====================================================================================
  */

}


// FUNCTIONS =======================================================================

int cross_correlation_lag(int8_t x1[], int8_t x2[]){
  // Function to calculate sample delay between two signals using cross-correlation
  // implemented using direct time-domain method
  // see https://github.com/trichter/sito/blob/master/src/xcorr.c for original c++ source code

  int a, a2, b, bmin, bmax, lag_max, shift, shift_zero;
  int cc_max;

  shift = 10;
  shift_zero = 0;
  int cc[2 * shift + 1];
  
  a = 0;
  a2 = -shift_zero - shift;

  // Main Cross-correlation calculation in nested for-loop
  for (; a < (2 * shift + 1); a++, a2++){
    bmin = max(0, -a2);
    bmax = min(N, -a2 + N);
    cc[a] = 0;

    if (bmin >= bmax){
      continue;
    }
    
    for (b = bmin; b < bmax; b++){
      cc[a] += (x1[b + a2] * x2[b])/100;
    }
    //Serial.println(cc[a]);
  }

  // Calculate sample delay between inputs using index (lag) value at maximum cross-correlation 
    lag_max = 0;
    cc_max = 0; //cc[lag_max];

    // Find max value in cc array
    for (int i = 1; i < (2 * shift + 1); i++){
      if (abs(cc[i]) > cc_max){
        cc_max = cc[i];
        lag_max = i;
      }
    }

  return lag_max - shift;
}

void normalise(int8_t data[]){
  // Normalises input values to standard level (assuming 8-bit resolution, 256 levels, +-128)
  // Modified from original Java source code found at:
  // https://github.com/kripthor/teensy31-micloc-java/blob/master/MicLocTeensy-Java/src/SoundUtils.java
  int max = 0;
  int targetMax = 127;

  //Find maximum value
  for (int i = 1; i < N; i++){
    int abs = abs(data[i]);
    if (abs > max){
      max = abs;
    }
  }
  double maxReduce = 1 - targetMax / double(max);

  for (int i = 0; i < N; i++){
    int abs = abs(data[i]);

    // Normalisation
    double factor = (maxReduce * abs / double(max));
    int dat = int((1 - factor) * data[i]);
    int absdat = abs(dat);
    if (absdat <= targetMax){
      data[i] = dat;
    }
  }
}

void driveForward(){  // set pins to make robot drive forward
  digitalWrite(leftMotorForwardPin,HIGH);
  digitalWrite(leftMotorBackwardPin,LOW);
  digitalWrite(rightMotorForwardPin,HIGH);
  digitalWrite(rightMotorBackwardPin,LOW);
}

void driveBackward(){  // set pins to make robot drive backward 
  digitalWrite(leftMotorForwardPin,LOW);
  digitalWrite(leftMotorBackwardPin,HIGH);
  digitalWrite(rightMotorForwardPin,LOW);
  digitalWrite(rightMotorBackwardPin,HIGH);
}

void turnRight(){  // set pins to make robot turn right
  digitalWrite(leftMotorForwardPin,HIGH);
  digitalWrite(leftMotorBackwardPin,LOW);
  digitalWrite(rightMotorForwardPin,LOW);
  digitalWrite(rightMotorBackwardPin,HIGH);
}

void turnLeft(){  // set pins to make robot turn left
  digitalWrite(leftMotorForwardPin,LOW);
  digitalWrite(leftMotorBackwardPin,HIGH);
  digitalWrite(rightMotorForwardPin,HIGH);
  digitalWrite(rightMotorBackwardPin,LOW);
}

void stopDriving(){  // set pins to make robot stop
  digitalWrite(leftMotorForwardPin,LOW);
  digitalWrite(leftMotorBackwardPin,LOW);
  digitalWrite(rightMotorForwardPin,LOW);
  digitalWrite(rightMotorBackwardPin,LOW);
}