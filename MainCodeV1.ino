#include <Arduino.h>
#include <math.h>
// Constant Variables for Microphone Pins //

const int mic1pin = A8;   // Right
const int mic2pin = A14;  // Left
const int mic3pin = A11;  // Rear Centre

/// Right Motor ///
const int enA = 10;
const int in1 = 3;
const int in2 = 2;

/// Left Motor ///
const int enB = 9;
const int in3 = 4;
const int in4 = 5;


const int RedLEDPin = 22;     // Red LED
const int YellowLEDPin = 23;  // Yellow LED
const int GreenLEDPin = 25;   // Green LED

//const int sampleWindow = 4000;  // time to take samples
int mic1;                       // Mic 1 Value
int mic2;                       // Mic 2 Value
int mic3;                       // Mic 3 Value

// Physical Constants //

float FS = 31250.0;  // sampling frequency approx
float d = 0.246;      // distance between mics
float Vs = 343.14;   // Speed of sound m/s


// Timing Values //

double tsm;  // measured sampling delta time (s)

unsigned long start_time;    // time in milliseconds at start of sample window (since program started)
unsigned long current_time;  // current time in milliseconds (since program started)

unsigned long sample_time_start;  // Time at start of N samples
unsigned long sample_time_end;    // Time at end of N samples over M channels

// Cross Correlation Calculation Variables //

const int N = 600;   // Number of samples per cc calculation
const int N3 = 400;  // Number of samples for mic 3 STE
float oneByN = 1.0 / N;
float oneByFS = 1.0 / FS;
float VsByd = Vs / d;
int count;  // Number of loops to calculate mean of cc values over time

float sample1[N];  // Input buffer array - Mic 1
float sample2[N];  // Input buffer array - Mic 2
float sample3[N3];  // Input buffer array - Mic 3

// Filter and Threshold Values //

float energyThreshold = 1000000.00;  // frame size 100 (6280000.00)  frame size 200 (12510000)  1285000.00      !!!!1265000.00
float crossingThresholdL = 0.10;
float crossingThresholdU = 0.3;      // !!!!0.16 swap sign in function!!!!!!!!!!!!!!!!!!!!!!!
const int FILTER_SIZE = 3;
float angle_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_full = false;
long rearThreshold = 3700000.00;  // threshold for mic 3 energy to change from phase difference to magnitude approach

const int MAX_ITERATIONS = 24;      // Adjust as needed based on expected max iterations
float angle_array[MAX_ITERATIONS];  // To store all collected angles

////////////////////////////// Motor Constants //////////////////////////////
float prevAngle;
int endCount = 0;
bool END;
int state = 'initial';

void setup() {


  // Initialise motor pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);


  // Set LED pins as output
  pinMode(RedLEDPin, OUTPUT);
  pinMode(YellowLEDPin, OUTPUT);
  pinMode(GreenLEDPin, OUTPUT);

  //TCCR2B=(TCCR2B&248)|2;    //prescaler=1 62500Hz

  // initialise ADC to increase max sampling rate to approx. 62.5 kHz
  ADCSRA = 0b10000100;    // set prescaler to 16 to increase sampling rate
  ADCSRB = 0b00000000;    // enable ADC free running mode
  ADMUX |= (1 << ADLAR);  // Left align ADC value (only read highest 8 bits)
  Serial.flush();
  Serial.begin(921600);  // initialize serial communication for debugging
  delay(5000);
}

void loop() {


  switch(state) {
    
    // Initial Movement //
    case 'initial':{
      Serial.println("Case 1");
      bool progress = false;
      //float test = maxSTE3();
      //Serial.print(test);
      
      while (!progress){
        progress = initialMove();
      }
      if (progress){
        //Serial.print("Transition to 3");
        state = 'turn';
      } 
      break;
    }
    // Check for proximity & end //
    case 'prox':{
      Serial.println("Case 2");
      
      bool close = false;
      
      digitalWrite(YellowLEDPin, HIGH);
      digitalWrite(GreenLEDPin, HIGH);
      
      //float currentAngle = collectAndAngle(9000);
      long currentIntensity = maxSTE3();
      
      close = RearVsFront(currentIntensity);

      digitalWrite(GreenLEDPin, LOW);
      digitalWrite(YellowLEDPin, LOW);
      
      //Serial.print(close);
      
      if (END || endCount >= 2){
        END = true;
        state = 'death';
      }else if(close){
        state = 'fine';
      }else{
        state = 'turn';  
      }
      break;
    } 
    // Turn and Move //
    case 'turn':{

      bool frontFace = false;
      
      Serial.println("Case 3");

      while (!frontFace){
        
        digitalWrite(GreenLEDPin, HIGH);
        
        float angle = collectAndAngle(9000);
        //Serial.println(angle);
        
        /*Serial.println("I turn now");
        delay(500);
        digitalWrite(GreenLEDPin, LOW);
        turnTowardsAngle(angle);
        delay(500);
        //Serial.println("Powerrrrrrrr");
        
        updateMotorsST(4000);
        delay(500);*/

        if (angle <= 4 && angle >= -4 ){
          frontFace = true;
          digitalWrite(GreenLEDPin, LOW);
          updateMotorsST(3000);

        }else{
          frontFace = false;
          delay(500);
          digitalWrite(GreenLEDPin, LOW);
          turnTowardsAngle(angle);
          delay(500);
          updateMotorsST(5000);
          delay(500);
        }
      }
      state = 'prox';
      break;
    }
    // Fine Movement //
    case 'fine':{
      Serial.println("Case 4");
      digitalWrite(YellowLEDPin, HIGH);
      digitalWrite(GreenLEDPin, HIGH);

      if (END || endCount >= 2){
        END = true;
        state = 'death';
        break;
      }else{
      }

      for (int x = 0; x < 1; x++){
        float initialMag = maxSTE3();
        delay(200);
        updateMotorsST(1500);
        delay(200);
        float newMag = maxSTE3();

        float magDif = newMag - initialMag;
        float absMagDif = abs(magDif);
        //Serial.println(magDif);

        if (newMag > 5000000){
          //Serial.println("DYINGGGGGGGG");
          endCount++;
          state = 'prox';
        }else{
          //Serial.println("LIVINGGGGGGG");  
          state = 'prox';
        }

      }
      digitalWrite(YellowLEDPin, LOW);
      digitalWrite(GreenLEDPin, LOW);
      break;
    }
    // End Condition Met //
    case 'death':{  
      Serial.println("Case 5");
      if (END){
        int die = 1;
        while (die == 1){
          updateMotorsHALT();
          digitalWrite(GreenLEDPin, LOW);
          digitalWrite(YellowLEDPin, LOW);
          digitalWrite(RedLEDPin, HIGH);
        }
      }
      break;
    }   
  }

  //collectAndAngle(100000);
  //maxSTE3();

}


////////////////////////////// FUNCTIONS //////////////////////////////

float maxSTE3(){
  
  float cIntensity = 0.0;
  float maxIntensity = 0.0; 
  for (int i = 0; i < 400; i++){
    cIntensity = STEMic3();

    if (cIntensity > maxIntensity){
      maxIntensity = cIntensity;
    }
  }
  //Serial.println(maxIntensity);
  return (maxIntensity);
}

float STEMic1() {

  for (int i = 0; i < N3; i++) {
    sample1[i] = analogRead(mic1pin) - 86;
  }

  /*for (int i = 0; i < N3; i++) {
    Serial.println(sample3[i]);
  }*/
  

  int frameSize1 = 25;  // Number of samples per frame. Increase value to reduce sensitivity to peaks
  int numFrames1 = N3 / frameSize1;
  float maxEnergy1 = 0.0;

  for (int i = 0; i < numFrames1; i++) {
    float energy1 = 0.0;
    for (int j = 0; j < frameSize1; j++) {
      int idx = i * frameSize1 + j;
      if (idx < N3) {
        energy1 += sample1[idx] * sample1[idx];
      }
    }
    if (energy1 > maxEnergy1) {
      maxEnergy1 = energy1;
    }
  }
  //Serial.println(maxEnergy3);
  return (maxEnergy1);
}

float maxSTE1(){
  
  float cIntensity = 0.0;
  float maxIntensity = 0.0; 
  for (int i = 0; i < 400; i++){
    cIntensity = STEMic1();

    if (cIntensity > maxIntensity){
      maxIntensity = cIntensity;
    }
  }
  //Serial.println(maxIntensity);
  return (maxIntensity);
}

float STEMic3() {

  for (int i = 0; i < N3; i++) {
    sample3[i] = analogRead(mic3pin) - 86;
  }

  /*for (int i = 0; i < N3; i++) {
    Serial.println(sample3[i]);
  }*/
  

  int frameSize3 = 25;  // Number of samples per frame. Increase value to reduce sensitivity to peaks
  int numFrames3 = N3 / frameSize3;
  float maxEnergy3 = 0.0;

  for (int i = 0; i < numFrames3; i++) {
    float energy3 = 0.0;
    for (int j = 0; j < frameSize3; j++) {
      int idx = i * frameSize3 + j;
      if (idx < N3) {
        energy3 += sample3[idx] * sample3[idx];
      }
    }
    if (energy3 > maxEnergy3) {
      maxEnergy3 = energy3;
    }
  }
  //Serial.println(maxEnergy3);
  return (maxEnergy3);
}

float cross_correlation_lag(float x1[], float x2[]) {

  int a2, b, lag_max, magshift;
  float bmin, shift, shift_zero, bmax, cc_max, threshold, lag_estimate;

  shift = 20;
  shift_zero = 0;
  int cc_size = 2 * shift + 1;
  float cc[cc_size];

  // Calculate threshold based on the maximum possible cross-correlation value
  float max_possible_value = 0;
  for (int i = 0; i < N; i++) {
    max_possible_value += (x1[i] * x2[i]);
  }
  threshold = abs(max_possible_value) * 0.35;  // set the percentage threshold

  // Main Cross-correlation calculation in nested for-loop
  for (int a = 0; a < (2 * shift + 1); a++) {
    int a2 = a - shift;
    bmin = max(0, -a2);
    bmax = min(N, N - a2);
    cc[a] = 0;

    if (bmin >= bmax) {
      continue;
    }
    for (b = bmin; b < bmax; b++) {
      cc[a] += (x1[b + a2] * x2[b]);
    }

    // Apply threshold
    if (abs(cc[a]) < threshold) {
      cc[a] = 0;
    }
    //Serial.println(cc[a]);
  }
  // Calculate sample delay between inputs using lag value at maximum cross-correlation
  lag_max = 0;
  cc_max = 0;  //cc[lag_max];

  // Find max value in cc array
  for (int i = 1; i < cc_size; i++) {
    if (abs(cc[i]) > abs(cc_max)) {
      cc_max = cc[i];
      lag_max = i;
    }
  }
  // Parametric Smoothing around Max Value
  if (lag_max > 0 && lag_max < cc_size - 1) {
    float y1 = cc[lag_max - 1];
    float y2 = cc[lag_max];
    float y3 = cc[lag_max + 1];

    float numerator = y1 - y3;
    float denominator = 2 * (y1 - 2 * y2 + y3);

    float delta = 0.0;
    if (denominator != 0) {
      delta = numerator / denominator;
    }
    lag_estimate = (lag_max - shift) + delta;
    // Use lag_estimate in subsequent calculations
  } else {
    lag_estimate = lag_max - shift;
  }
  //lag_estimate = lag_max - shift;
  //Serial.print("Calculated lag: ");
  //Serial.println(lag_estimate);

  // Verify if lag is within physically possible limits
  float max_lag = ((d / Vs) * FS);
  if (abs(lag_estimate) > max_lag) {
    Serial.println("Lag exceeds maximum possible value. Check signals and cross-correlation.");
    // Handle error accordingly
  }

  // Calculate time delay
  float delta_time = lag_estimate * oneByFS;

  // Adjust for ADC read delay
  float adc_read_delay = 24e-6;
  float delta_t_adjusted = delta_time - adc_read_delay;

  // Print adjusted time delay
  //Serial.print("Time delay (delta_t): ");
  //Serial.println(delta_t, 8);
  //Serial.print("Adjusted time delay (delta_t_adjusted): ");
  //Serial.println(delta_t_adjusted, 8);

  // Calculate argument for asin, ensure it is within valid range
  float argument = VsByd * delta_t_adjusted;
  //Serial.print("Argument for asin: ");
  //Serial.println(argument, 6);

  // Check if argument is within valid range
  if (argument > 1.0) {
    Serial.println("Argument exceeds 1.0, setting to 1.0");
    argument = 1.0;
  } else if (argument < -1.0) {
    Serial.println("Argument is less than -1.0, setting to -1.0");
    argument = -1.0;
  }

  // Calculate angle in radians
  float theta = asin(argument);

  // Convert angle to degrees
  float angle_degrees = theta * (180.0 / PI);

  Serial.print("Calculated angle: ");
  Serial.println(angle_degrees);

  //return angle;
  return angle_degrees;
}

void normalise(float data[], int N) {
  float maxV = data[0];
  float minV = data[0];
  float targetMax = 127;
  float targetMin = -127;
  float range = 0;

  // Find maximum and minimum values
  for (int i = 0; i < N; i++) {
    if (data[i] > maxV) maxV = data[i];
    if (data[i] < minV) minV = data[i];
  }

  range = maxV - minV;

  // Normalize if range is sufficient
  if (range >= 3) {
    float scale = (targetMax - targetMin) / range;
    for (int i = 0; i < N; i++) {
      data[i] = (data[i] - minV) * scale + targetMin;

      //data[i] = map(data[i], minV, maxV, targetMin, targetMax);
    }
  }
}

void zeroMean(float data[], int N) {
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += data[i];
  }
  float mean = sum / (float)N;
  for (int i = 0; i < N; i++) {
    data[i] -= mean;
  }
}

float angleErrorCorrection(float theta) {

  float coeffs[] = { -3e-8, 3e-7, 6e-5, -0.0012, -0.067, 0.0 };

  // Polynomial to adjust

  float error = theta * (theta * (theta * (theta * (theta * (coeffs[0]) + coeffs[1]) + coeffs[2]) + coeffs[3]) + coeffs[4]) + coeffs[5];
  float correctedAngle = theta - error;
  return correctedAngle;
}

bool isSoundPlayingSTE(float data[], int N, float energyThreshold) {
  int frameSize = 20;  // Increase value to reduce sensitivity to peaks
  int numFrames = N / frameSize;
  float maxEnergy = 0.0;

  for (int i = 0; i < numFrames; i++) {
    float energy = 0.0;
    for (int j = 0; j < frameSize; j++) {
      int idx = i * frameSize + j;
      if (idx < N) {
        energy += data[idx] * data[idx];
      }
    }
    if (energy > maxEnergy) {
      maxEnergy = energy;
    }
  }
  //Serial.println(maxEnergy);
  return (maxEnergy > energyThreshold);
}

float initialSTE(float data[], int N) {
  int frameSize = 20;  // Increase value to reduce sensitivity to peaks
  int numFrames = N / frameSize;
  float maxEnergy = 0.0;

  for (int i = 0; i < numFrames; i++) {
    float energy = 0.0;
    for (int j = 0; j < frameSize; j++) {
      int idx = i * frameSize + j;
      if (idx < N) {
        energy += data[idx] * data[idx];
      }
    }
    if (energy > maxEnergy) {
      maxEnergy = energy;
    }
  }

  return (maxEnergy);
}

void updateAngleBuffer(float new_angle) {
  angle_buffer[buffer_index] = new_angle;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;
  if (buffer_index == 0) {
    buffer_full = true;
  }
}

float computeStandardDeviation(float data[], int size, float mean) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += pow(data[i] - mean, 2);
  }
  return sqrt(sum / size);
}

float computeMean(float data[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum / size;
}

void removeOutliers(float data[], int& size) {
  if (size <= 2) {
    // Not enough data
    //Serial.println("Not enough values before outlier removal");
    return;
  }

  float mean = computeMean(data, size);
  float sd = computeStandardDeviation(data, size, mean);
  float threshold = 2.5 * sd;  // adjust coefficient depending on sens required

  // Create a new array for filtered data
  float filteredData[size];
  int filteredSize = 0;

  for (int i = 0; i < size; i++) {
    if (abs(data[i] - mean) <= threshold) {
      filteredData[filteredSize++] = data[i];
    }
  }
  // Filtered data back to array
  for (int i = 0; i < filteredSize; i++) {
    data[i] = filteredData[i];
  }
  size = filteredSize;
}

float computeMovingAverage() {
  int count = buffer_full ? FILTER_SIZE : buffer_index;
  //Serial.print(count);
  float sum = 0.0;
  for (int i = 0; i < count; i++) {
    sum += angle_buffer[i];
  }
  return sum / count;
}

float computeMedian(float data[], int size) {
  float temp[size];
  memcpy(temp, data, size * sizeof(float));

  // bubble sort
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[i] > temp[j]) {
        float swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  // Compute median
  if (size % 2 == 0) {
    // Even
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
  } else {
    // Odd
    return temp[size / 2];
  }
}

bool isSoundPlayingZCR(float data[], int N, float thresholdL, float thresholdU) {
  int zeroCrossings = 0;
  for (int i = 1; i < N; i++) {
    if ((data[i - 1] >= 0 && data[i] < 0) || (data[i - 1] < 0 && data[i] >= 0)) {
      zeroCrossings++;
    }
  }
  float zcr = (float)zeroCrossings / N;
  //Serial.print("Z1:");
  //Serial.println(zcr);
  return ((zcr < thresholdU) && (zcr > thresholdL));
}

float updateMotorsST(float travelTime) {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);


  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  bool Timer = false;

  long turnTime = travelTime;

  unsigned long sMotorTime = millis();
  unsigned long cMotorTime = sMotorTime;

  while (!Timer) {
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 1");
    } else {
      Timer = false;
      cMotorTime = millis();
      //Serial.println(cMotorTime);
    }
  }

  updateMotorsHALT();
}

void updateMotorsHALT() {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
}

float updateMotorsRT(float angle) {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);

  bool Timer = false;


  long turnTime = turningTime(angle);

  unsigned long sMotorTime = millis();
  unsigned long cMotorTime = sMotorTime;

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, HIGH);
  digitalWrite(enB, LOW);

  while (!Timer) {
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 1");
    } else {
      Timer = false;
      cMotorTime = millis();
      //Serial.println(cMotorTime);
    }
  }

  updateMotorsHALT();
}

float updateMotorsLT(float angle) {
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);

  bool Timer = false;


  long turnTime = turningTime(angle);

  unsigned long sMotorTime = millis();
  unsigned long cMotorTime = sMotorTime;

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, LOW);
  digitalWrite(enB, HIGH);

  while (!Timer) {
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 1");
    } else {
      Timer = false;
      cMotorTime = millis();
      //Serial.println(cMotorTime);
    }
  }

  updateMotorsHALT();
}

bool angleCompare(float angle) {
  bool validTurn;
  //angle = abs(angle);
  //prevAngle = abs(prevAngle);
  float angleChange = angle - prevAngle;
  //Serial.print("Angle Change: ");
  //Serial.println(angleChange);
  if (angleChange > 40 || angleChange < -40) {
    validTurn = false;
  } else if (angleChange <= 40 || angleChange >= -40) {
    validTurn = true;
  }

  if (angleChange <= 40 || angleChange >= -40) {
    prevAngle = angle;
  }
  return (validTurn);
}

float collectAndAngle(int sampleWindow) {
  //Serial.flush();
  count = 0;  // Set/reset averaging counter
  start_time = 0;
  current_time = 0;

  /*for (int i = 0; i < N; i++){
    sample1[i] = 0;
    sample2[i] = 0;
  }*/

  // Initialise time current loop
  start_time = millis();
  current_time = millis();

  // Read microphone 1&2
  while (current_time - start_time < sampleWindow) {
    //Serial.println(current_time - start_time);

    for (int i = 0; i < N; i++) {
      sample1[i] = analogRead(mic1pin) - 86;
      sample2[i] = analogRead(mic2pin) - 86;
    }

    /*for (int i = 0; i < N; i++){
      Serial.print("S1:");
      Serial.println(sample1[i]);
      Serial.print("S2:");
      Serial.println(sample2[i]);
      Serial.print("S3:");
      Serial.println("350");
      Serial.print("S4:");
      Serial.println("150");
      
    }*/

    // Check if sound is playing
    if (isSoundPlayingSTE(sample1, N, energyThreshold) && isSoundPlayingSTE(sample2, N, energyThreshold)) {
      // normalise sampled data
      normalise(sample1, N);
      normalise(sample2, N);

      zeroMean(sample1, N);
      zeroMean(sample2, N);
    } else {
      //Serial.println("STE");

      current_time = millis();
      continue;
    }
    if (isSoundPlayingZCR(sample1, N, crossingThresholdL, crossingThresholdU) && isSoundPlayingZCR(sample2, N, crossingThresholdL, crossingThresholdU)) {

      //Serial.println("Sound playing");
      // CC angle calculation and error correction
      float angle = angleErrorCorrection(cross_correlation_lag(sample1, sample2));

      // Add angle to array
      if (count < MAX_ITERATIONS && (angleCompare(angle) && count >= 2)) {
        updateAngleBuffer(angle);
        angle_array[count] = angle;
        //Serial.println(angle);
        count++;  // increment counter for averaging
        //Serial.println(count);
      } else if (count < MAX_ITERATIONS && count < 2){
        updateAngleBuffer(angle);
        angle_array[count] = angle;
        count++;
      }else{
        //Serial.println("Warning: Exceeded maximum iterations. Consider increasing MAX_ITERATIONS.");
        //break;
      }
    } else {
      //Serial.println("Volume too low; skipping this iteration.");
      //Serial.println("ZCR");
      //digitalWrite(GreenLEDPin, LOW);
      current_time = millis();
      continue;
    }
    current_time = millis();  // time at end of averaging
  }
  /*for (int i = 0 ; i < count ; i++){
    Serial.print(angle_array[i]);
  }*/
  removeOutliers(angle_array, count);

  // Check if enough data remains after outlier removal
  /*if (count < 2) {
    //Serial.println("No valid angles after outlier removal.");
    // Handle as appropriate
    return;
  }*/

  // Mean vs Moving average depending on testing results
  //float median_angle = computeMedian(angle_array, count);
  float filtered_angle = computeMovingAverage();
  Serial.print("Filtered Angle: ");
  Serial.println(filtered_angle);
  return (filtered_angle);
}

void turnTowardsAngle(float angle) {

  if (angle <= -5) {
    updateMotorsLT(angle);
    Serial.println("LT");
  } else if (angle >= 5) {
    updateMotorsRT(angle);
    Serial.println("RT");
  } else {
    //updateMotors();
    Serial.print("ST");
  }
}

bool RearVsFront(long STEnergy) {
  //angle = abs(angle);

  if ( (STEnergy > rearThreshold)) {
    return true;
  } else {
    return false;
  }
}

bool initialMove() {
  int maxD;
  
  digitalWrite(GreenLEDPin, HIGH);
  //float angle1 = collectAndAngle(9000);
  float STE1 = maxSTE1();
  digitalWrite(GreenLEDPin, LOW);
  
  turnTowardsAngle(120);
  delay(500);

  digitalWrite(GreenLEDPin, HIGH);
  //float angle2 = collectAndAngle(9000);
  float STE2 = maxSTE1();
  digitalWrite(GreenLEDPin, LOW);

  turnTowardsAngle(120);
  delay(500);

  digitalWrite(GreenLEDPin, HIGH);
  //float angle3 = collectAndAngle(9000);
  float STE3 = maxSTE1();
  digitalWrite(GreenLEDPin, LOW);


  if (STE1 >= STE2 && STE1 >= STE3){
    maxD = 1;
    Serial.print(maxD);
    turnTowardsAngle(120);
    return true;
  }else if(STE2 >= STE1 && STE2 >= STE3){
    maxD = 2;
    Serial.print(maxD);
    turnTowardsAngle(120);
    delay(500);
    turnTowardsAngle(120);
    return true;
  }else if(STE3 >= STE1 && STE3 >= STE2){
    maxD = 3;
    Serial.print(maxD);
    return true;
  }else{
    return false;
  }

  /*int i = 0;
  while (i < 1) {
    int Iangle = collectAndAngle(2000);
    int absIangle = abs(Iangle);
    //Serial.println(Iangle);
    if (absIangle < 40 && absIangle > 5) {
      i = 1;
      return true;
    } else {
      //Serial.print("Initial turn now");
      turnTowardsAngle(140);
      i = 0;
    }
  }*/
}

float turningTime(float angle) {

  float coeffs[] = {2e-10, 1e-9, -9e-6, -5e-5, 0.2082, 0.3766, 200.0};

  float Time = angle * (angle * (angle * (angle * (angle * (angle * (coeffs[0]) + coeffs[1]) + coeffs[2]) + coeffs[3]) + coeffs[4]) + coeffs[5]) + coeffs[6];
  //Serial.println("Turn time: ");
  //Serial.print(Time);
  return (Time);
}