const int Mic1 = A1;
const int Mic2 = A2;
const int HighB = -8000;
const int LowB = -20000;
int micReading1 = 0;
int micReading2 = 0;
int micReading1Amp = 0;
int micReading2Amp = 0;


void setup() {
  // put your setup code here, to run once:
    // initialise ADC to increase max sampling rate to approx. 62.5 kHz
  ADCSRA = 0b10000100;  // set prescaler to 16 to increase sampling rate
  ADCSRB = 0b00000000;  // enable ADC free running mode
  ADMUX |= (1 << ADLAR);  // Left align ADC value (only read highest 8 bits)
  Serial.begin(921600);
}

void loop() {
  // put your main code here, to run repeatedly:
  micReading1 = analogRead(Mic1);
  micReading2 = analogRead(Mic2);
  micReading1Amp = sq(micReading1);
  micReading2Amp = sq(micReading2);

  Serial.print("Variable_1:");
  Serial.print(micReading1Amp);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(micReading2Amp);
  Serial.print("Const_1:");
  Serial.print(HighB);
  Serial.print(",");
  Serial.print("Const_2:");
  Serial.println(LowB);
}
