#include <Arduino.h>
#include <math.h>
// Constant Variables for Microphone Pins //

const int mic1pin = A9; // Right
const int mic2pin = A14; // Left
const int mic3pin = A11; // Rear Centre

/// Right Motor ///
const int enA = 10;
const int in1 = 3;
const int in2 = 2;

/// Left Motor ///
const int enB = 9;
const int in3 = 4;
const int in4 = 5;


const int RedLEDPin = 22; // Red LED
const int YellowLEDPin = 23; // Yellow LED
const int GreenLEDPin = 25; // Green LED

const int sampleWindow = 3000; // time to take samples
int mic1; // Mic 1 Value
int mic2; // Mic 2 Value
int mic3; // Mic 3 Value

// Physical Constants //

float FS = 31250.0; // sampling frequency approx
float d = 0.25; // distance between mics
float Vs = 343.14; // Speed of sound m/s


// Timing Values //

double tsm;                       // measured sampling delta time (s)

unsigned long start_time;         // time in milliseconds at start of sample window (since program started)
unsigned long current_time;       // current time in milliseconds (since program started)

unsigned long sample_time_start;  // Time at start of N samples
unsigned long sample_time_end;    // Time at end of N samples over M channels

// Cross Correlation Calculation Variables //

const int N = 400;        // Number of samples per cc calculation
const int N3 = 400;       // Number of samples for mic 3 STE 
float oneByN = 1.0 / N;
float oneByFS = 1.0 / FS;
float VsByd = Vs / d;
int count;                // Number of loops to calculate mean of cc values over time

float sample1[N];        // Input buffer array - Mic 1
float sample2[N];        // Input buffer array - Mic 2
float sample3[N];        // Input buffer array - Mic 3

// Filter and Threshold Values //

float energyThreshold = 1265000.00 ; // frame size 100 (6280000.00)  frame size 200 (12510000)  1285000.00
float crossingThreshold = 0.16;
const int FILTER_SIZE = 3;
float angle_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_full = false;
long rearThreshold = 1270000.00; // threshold for mic 3 energy to change from phase difference to magnitude approach

const int MAX_ITERATIONS = 24; // Adjust as needed based on expected max iterations
float angle_array[MAX_ITERATIONS]; // To store all collected angles

////////////////////////////// Motor Constants //////////////////////////////
int baseSpeed = 10; // Base Speed that the motor will run at when not altered
int kp = 3; // Constant of Proportionality - (Adjust to change how aggressive motor angle change is)
float prevAngle;
float direction;

int index = 0;

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

  //TCCR2B=(TCCR2B&248)|2;    //prescaler=1 62500Hz, 2 ~8000Hz 

  // initialise ADC to increase max sampling rate to approx. 62.5 kHz
  ADCSRA = 0b10000100;  // set prescaler to 16 to increase sampling rate
  ADCSRB = 0b00000000;  // enable ADC free running mode
  ADMUX |= (1 << ADLAR);  // Left align ADC value (only read highest 8 bits)
  Serial.flush();
  Serial.begin(921600);  // initialize serial communication for debugging
  
  //delay(15000);
}


void loop() {

  // STE Thresholds:
  // go to isSoundPlayingSTE and uncomment print
  // adjust thresholds above
  //collectAndAngle();

  // ZCR Thresholds:
  // go to isSoundPlayingZCR and uncomment print
  // adjust thresholds above
  //collectAndAngle();

  // Angle Error correction:
  // make sure nothing else is printing
  // test with motors connected but halted
  collectAndAngle();

  // Motor Turn time:
  // go to function and adjust turn time as needed
  // check wiring and switch if L&R reversed then change code. if F&B reversed change wiring

  /*if (index < 1){
    
    //Right turn or left dependent on wiring
    updateMotorsRT();
    //updateMotorsLT();


    // ST function is endless add timer from LT or RT to limit if needed
    //updateMotorsST();
    index = 5;
  }else{
    updateMotorsHALT();
  }*/



}

float STEMic3(){

  for (int i = 0; i < N3; i++){
    sample3[i] = analogRead(mic3pin)-86;
  }
  
  int frameSize3 = 20;  // Number of samples per frame. Increase value to reduce sensitivity to peaks
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
  return (maxEnergy3);
}

float initialSTE(float data[], int N){
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

void normalise(float data[], int N) {
  float maxV = data[0];
  float minV = data[0];
  float targetMax = 127;
  float targetMin = -127;
  float range = 0;

  // Find maximum and minimum values
  for (int i = 1; i < N; i++) {
    if (data[i] > maxV) maxV = data[i];
    if (data[i] < minV) minV = data[i];
  }

  range = maxV - minV;

  // Normalize if range is sufficient
  if (range >= 8) {
    float scale = (targetMax - targetMin) / range;
    for (int i = 0; i < N; i++) {
      data[i] = (data[i] - minV)* scale + targetMin;
      
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

bool isSoundPlayingZCR(float data[], int N, float threshold) {
  int zeroCrossings = 0;
  for (int i = 1; i < N; i++) {
    if ((data[i - 1] >= 0 && data[i] < 0) || (data[i - 1] < 0 && data[i] >= 0)) {
      zeroCrossings++;
    }
  }
  float zcr = (float)zeroCrossings / N;
  //Serial.println(zcr);
  return (zcr < threshold);
}

float cross_correlation_lag(float x1[], float x2[]){

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
  threshold = abs(max_possible_value) * 0.5; // set the percentage threshold

  // Main Cross-correlation calculation in nested for-loop
  for (int a = 0; a < (2 * shift + 1); a++){
    int a2 = a - shift;
    bmin = max(0, -a2);
    bmax = min(N, N - a2);
    cc[a] = 0;

    if (bmin >= bmax){
      continue;
    }
    for (b = bmin; b < bmax; b++){
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
  cc_max = 0; //cc[lag_max];

  // Find max value in cc array
  for (int i = 1; i < cc_size; i++){
    if (abs(cc[i]) > abs(cc_max)){
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
  } else{
    lag_estimate = lag_max - shift;
  }      
  //Serial.print("Calculated lag: ");
  //Serial.println(lag_estimate);

  // Verify if lag is within physically possible limits
  float max_lag = ((d / Vs) * FS) ;
  if (abs(lag_estimate) > max_lag) {
    Serial.println("Lag exceeds maximum possible value. Check signals and cross-correlation.");
    // Handle error accordingly
  }

  // Calculate time delay
  float delta_t = lag_estimate * oneByFS;

  // Adjust for ADC read delay
  float adc_read_delay = 24e-6; 
  float delta_t_adjusted = delta_t - adc_read_delay;

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
    Serial.println("Not enough values before outlier removal");
    return;
  }

  float mean = computeMean(data, size);
  float sd = computeStandardDeviation(data, size, mean);
  float threshold = 2 * sd;  // adjust coefficient depending on sens required

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

float angleErrorCorrection (float theta){

  //float coeffs[] = {-3e-8, 3e-7, 6e-5, -0.0012, -0.067, 0.0};

  float coeffs[] = {-8e-9, 8e-7, 3e-5, -0.0028, -0.2342, 0.0};
  // Polynomial to adjust

  float error = theta *(theta *(theta *(theta *(theta *(coeffs[0])+coeffs[1])+coeffs[2])+coeffs[3])+coeffs[4])+coeffs[5];
  float correctedAngle = theta - error;
  return correctedAngle;
}  

bool angleCompare(float angle){  
  angle = abs(angle);
  prevAngle = abs(prevAngle);
  float angleChange = angle - prevAngle;

  if (angleChange > 30){
    prevAngle = angle;
    return false;

  } else if (angleChange <= 30){
    prevAngle = angle;
    return true;
  }
}

float collectAndAngle(){
  //Serial.flush();
  count = 0;    // Set/reset averaging counter
  start_time = 0;
  current_time = 0;

  /*for (int i = 0; i < N; i++){
    sample1[i] = 0;
    sample2[i] = 0;
  }*/
  digitalWrite(GreenLEDPin, LOW);
  // Initialise time current loop
  start_time = millis();
  current_time = millis();

  // Read microphone 1&2
  while( current_time - start_time < sampleWindow){
    //Serial.println(current_time - start_time);
    for (int i = 0; i < N; i++){
      sample1[i] = analogRead(mic1pin)-86;
      sample2[i] = analogRead(mic2pin)-86;
    }

    /*for (int i = 0; i < N; i++){
      Serial.println(sample1[i]);
      //Serial.println(sample2[i]);
    }*/
    
    // Check if sound is playing
    if (isSoundPlayingSTE(sample1, N, energyThreshold) || isSoundPlayingSTE(sample2, N, energyThreshold)){
      // normalise sampled data
      normalise(sample1, N);
      normalise(sample2, N);

      zeroMean(sample1, N);
      zeroMean(sample2, N); 
    } else{
      //Serial.println("STE");
      
      current_time = millis();
      continue;
    }
    if (isSoundPlayingZCR(sample1, N, crossingThreshold) || isSoundPlayingZCR(sample2, N, crossingThreshold)){
      // Green LED on when listening
      
      //Serial.println("Sound playing");
      // CC angle calculation and error correction
      float angle = angleErrorCorrection(cross_correlation_lag(sample1,sample2));    
      digitalWrite(GreenLEDPin, HIGH); 
      
      // Add angle to array
      if (count < MAX_ITERATIONS) {
        updateAngleBuffer(angle);
        angle_array[count] = angle;
        Serial.println(angle);
        count++;  // increment counter for averaging
        //Serial.println(count);
      } else {
        Serial.println("Warning: Exceeded maximum iterations. Consider increasing MAX_ITERATIONS.");
        break;
      }      
    } else{
      //Serial.println("Volume too low; skipping this iteration.");
      //Serial.println("ZCR");
      
      current_time = millis();
      continue;
    }
    current_time = millis();    // time at end of averaging
  }
  /*for (int i = 0 ; i < count ; i++){
    Serial.print(angle_array[i]);
  }*/
  removeOutliers(angle_array, count);

  // Check if enough data remains after outlier removal
  if (count < 2 ) {
    Serial.println("No valid angles after outlier removal.");
    // Handle as appropriate
    return;
  }
  // Mean vs Moving average depending on testing results
  float median_angle = computeMedian(angle_array, count);
  float filtered_angle = computeMovingAverage();
  //Serial.println(filtered_angle);

  //Angle sanity check for excessive change
  /*if (!(angleCompare(filtered_angle))){
    return;
    }else{
      //Serial.println(filtered_angle);
    }
  return filtered_angle;*/
}

///// MOTOR CONTROL FUNCTIONS /////

void updateMotorsHALT(){
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

void updateMotorsST(){
  digitalWrite(in2, HIGH);
	digitalWrite(in1, LOW);
	digitalWrite(in4, HIGH);
	digitalWrite(in3, LOW);

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  
}

void updateMotorsLT(){
  digitalWrite(in2, HIGH);
	digitalWrite(in1, LOW);
	digitalWrite(in4, HIGH);
	digitalWrite(in3, LOW);

  bool Timer = false;


  long turnTime = 2000;
  
  unsigned long sMotorTime = millis();
  unsigned long cMotorTime = sMotorTime;

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, HIGH);
  digitalWrite(enB, LOW);

  while (!Timer){
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 1");
    }else{
      Timer = false;
      cMotorTime = millis();
      //Serial.println(cMotorTime);
    }
  }

  updateMotorsHALT();
  
}

void updateMotorsRT(){
  digitalWrite(in2, HIGH);
	digitalWrite(in1, LOW);
	digitalWrite(in4, HIGH);
	digitalWrite(in3, LOW);

  bool Timer = false;


  long turnTime = 2000;
  
  unsigned long sMotorTime = millis();
  unsigned long cMotorTime = sMotorTime;

  /*int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);*/

  digitalWrite(enA, LOW);
  digitalWrite(enB, HIGH);

  while (!Timer){
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 1");
    }else{
      Timer = false;
      cMotorTime = millis();
      //Serial.println(cMotorTime);
    }
  }
  
  updateMotorsHALT();
  
}

// Analogwrite based motor functions //

void updateMotors(int adjustment){
  digitalWrite(in2, HIGH);
	digitalWrite(in1, LOW);
	digitalWrite(in4, HIGH);
	digitalWrite(in3, LOW);

  int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment;

  analogWrite(enA,SpeedR);
  analogWrite(enB,SpeedL);
}

void moveTowardsAngle(float angle){


  /*if (angle <= 5){
    return;
  } else{
  }*/
  //float anglecons = constrain(angle,-90, 90);  // Ensure angle doesnt exceed maximum 
  int adjustment = angle * kp;
  int SpeedR = baseSpeed + adjustment;
  int SpeedL = baseSpeed - adjustment; 
  bool Timer = false;


  long turnTime = 100;
  
  //unsigned long sMotorTime = millis();
  //unsigned long cMotorTime = sMotorTime;


  updateMotors(adjustment);


  /*while (!Timer){
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 2");
      
    }
      else{
        Timer = false;
        cMotorTime = millis();
    }
  }

  updateMotors(-adjustment);

  Timer = false;
  sMotorTime = millis();
  cMotorTime = sMotorTime;
  
  while (!Timer){
    if ((cMotorTime - sMotorTime) >= turnTime) {
      Timer = true;
      //Serial.print("Time Done 2");
      
    }
      else{
        Timer = false;
        cMotorTime = millis();
    }
  }
  //Serial.print("Function Done");


  /*int angleR = 0;
  angleR = angle;
  Serial.print(angleR);
  if (angleR < -5){
    updateMotorsRT();
    Serial.print("RT");
  } else if (angleR > 5){
      updateMotorsLT();
      Serial.print("LT");
  } else{
      updateMotorsST();
      Serial.print("ST");
  }*/

}
