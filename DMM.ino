#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif



// number of analog samples to take per reading
//#define NUM_SAMPLES 1

//long sum = 0;                    // sum of samples taken
//unsigned char sample_count = 0; // current sample number
//float voltage = 0.0;            // calculated voltage


// Voltage Mode:
// 0 = Analog
// 1 = Digital
int VOLTAGE_MODE = 0;

void setup()
{
  #if FASTADC
    // set prescale to 16
    sbi(ADCSRA,ADPS2) ;
    cbi(ADCSRA,ADPS1) ;
    cbi(ADCSRA,ADPS0) ;
  #endif

    Serial.begin(1000000);
    //Serial.begin(9600);
}

int sampleDelay = 0;

void loop()
{  
    //int delayed_time = 0;
    if (VOLTAGE_MODE == 0)
    {
      //writeIntAsBinaryt(analogRead(A2));
      
      int val = analogRead(A5);
      
      Serial.write((byte)(map(val, 0, 1023, 0, 255)));
      
      /*int Negval = analogRead(A3);
      
      Serial.write((byte)(map(Negval, 0, 1023, 0, 255)));*/
      
      if (sampleDelay != 0)
        delayMicroseconds(sampleDelay);
        
      //Serial.write((byte)highByte(val));
      //Serial.write((byte)255);
      
      //Serial.print((int)analogRead(A2));
      //delay(1);
      //delayMicroseconds(10);
    }
    else if (VOLTAGE_MODE == 1)
    {
      Serial.write((byte)(bitRead(PINB,0)));
      
      //delay(100);
      
      //unsigned char v = PIND>>2;  // remove tx/rx lines

      //Serial.write(v);
      
      if (sampleDelay > 0)
        delayMicroseconds(sampleDelay);
    }
    
    // take a number of analog samples and add them up
    /*while (sample_count < NUM_SAMPLES) {
        sum += analogRead(A2);
        sample_count++;
        delay(1);
        delayed_time++;
    }
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 5.015V is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 4.90) / 1024.0;
    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.132 is the calibrated voltage divide
    // value
    Serial.println(voltage * 11.80);
    sample_count = 0;
    sum = 0;
    
    if (delayed_time < 20)
      delay(20 - delayed_time);*/
}

void serialEvent()
{
 while(Serial.available() > 1)
  {
    byte tmp_sample = Serial.read();
    byte tmp_voltmode = Serial.read();
    char lineCar = Serial.read();
    
    if (lineCar == '\n' && tmp_voltmode < 2)
    {
       sampleDelay = tmp_sample;
       VOLTAGE_MODE = tmp_voltmode; 
    }
  } 
}
