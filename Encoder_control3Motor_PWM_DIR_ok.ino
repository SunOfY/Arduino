/*  ENCODER CODE
 DA KIEM TRA CODE OK   */

// Motor encoder 01
const int pinA0  = 2;
const int pinB0  = 3;

const int pinPWM0 = 5;
const int pinDIR0 = 32;

int pos0 = 0;

// Motor encoder 02
const int pinA1  = 20;
const int pinB1  = 21;

const int pinPWM1 = 7;
const int pinDIR1 = 34;

int pos1 = 0;

// Motor encoder 03
const int pinA2  = 19;
const int pinB2  = 18;

const int pinPWM2 = 9;
const int pinDIR2 = 36;

int pos2 = 0;

unsigned char duty[20] = {0, };
unsigned char duty0[4];
unsigned char duty1[4];
unsigned char duty2[4];
long PWM0, PWM1, PWM2;

  
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int getIntNum(int pin) {
  //returns the interrupt number for a given interrupt pin. see http://arduino.cc/it/Reference/AttachInterrupt 
  //ARDUINO MAGE 2560
switch(pin) {

  case 2:
    return 0;
  case 3:
    return 1;
  case 20:
    return 2;
  case 21:
    return 3;
  case 19:
    return 4;
  case 18:
    return 5;   
  default:
    return -1;
  }
}

void debounce(int del) {
  
  for (int k=0;k<del;k++) {
    /* can't use delay in the ISR so need to waste some time
       perfoming operations, this uses roughly 0.1ms on uno  */
    k = k +0.0 +0.0 -0.0 +3.0 -3.0 + 5 - 5 + 6 - 6;
  }
}

//ENDCODER 0==========================================
void irsPinAEn0(){

  /* read pin B right away                                   */
  int drB = digitalRead(pinB0);
  
  /* possibly wait before reading pin A, then read it        */
  debounce(0);
  int drA = digitalRead(pinA0);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */
      
    if (drB == LOW) {  /* check pin B */
    pos0++;  /* going clockwise: increment         */
    } else {
    pos0--;  /* going counterclockwise: decrement  */
    }
    
  } else {                       /* must be high to low on A */
  
    if (drB == HIGH) { /* check pin B */
    pos0++;  /* going clockwise: increment         */
    } else {
    pos0--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update                                    */

} /* end ISR pin A Encoder 0                                 */



/* Interrupt Service Routine: change on pin B for Encoder 0  */
void isrPinBEn0(){ 

  /* read pin A right away                                   */
  int drA = digitalRead(pinA0);
  
  /* possibly wait before reading pin B, then read it        */
  debounce(0);
  int drB = digitalRead(pinB0);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */
  
    if (drA == HIGH) { /* check pin A */
    pos0++;  /* going clockwise: increment         */
    } else {
    pos0--;  /* going counterclockwise: decrement  */
    }
  
  } else {                       /* must be high to low on B */
  
    if (drA == LOW) {  /* check pin A */
    pos0++;  /* going clockwise: increment         */
    } else {
    pos0--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update */

} /* end ISR pin B Encoder 0  */

//ENDCODER 1==========================================
void irsPinAEn1(){

  /* read pin B right away                                   */
  int drB = digitalRead(pinB1);
  
  /* possibly wait before reading pin A, then read it        */
  debounce(0);
  int drA = digitalRead(pinA1);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */
      
    if (drB == LOW) {  /* check pin B */
    pos1++;  /* going clockwise: increment         */
    } else {
    pos1--;  /* going counterclockwise: decrement  */
    }
    
  } else {                       /* must be high to low on A */
  
    if (drB == HIGH) { /* check pin B */
    pos1++;  /* going clockwise: increment         */
    } else {
    pos1--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update                                    */

} /* end ISR pin A Encoder 0                                 */



/* Interrupt Service Routine: change on pin B for Encoder 0  */
void isrPinBEn1(){ 

  /* read pin A right away                                   */
  int drA = digitalRead(pinA1);
  
  /* possibly wait before reading pin B, then read it        */
  debounce(0);
  int drB = digitalRead(pinB1);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */
  
    if (drA == HIGH) { /* check pin A */
    pos1++;  /* going clockwise: increment         */
    } else {
    pos1--;  /* going counterclockwise: decrement  */
    }
  
  } else {                       /* must be high to low on B */
  
    if (drA == LOW) {  /* check pin A */
    pos1++;  /* going clockwise: increment         */
    } else {
    pos1--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update */

} /* end ISR pin B Encoder 0  */

//ENDCODER 2==========================================
void irsPinAEn2(){

  /* read pin B right away                                   */
  int drB = digitalRead(pinB2);
  
  /* possibly wait before reading pin A, then read it        */
  debounce(0);
  int drA = digitalRead(pinA2);

  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */
      
    if (drB == LOW) {  /* check pin B */
    pos2++;  /* going clockwise: increment         */
    } else {
    pos2--;  /* going counterclockwise: decrement  */
    }
    
  } else {                       /* must be high to low on A */
  
    if (drB == HIGH) { /* check pin B */
    pos2++;  /* going clockwise: increment         */
    } else {
    pos2--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update                                    */

} /* end ISR pin A Encoder 0                                 */



/* Interrupt Service Routine: change on pin B for Encoder 0  */
void isrPinBEn2(){ 

  /* read pin A right away                                   */
  int drA = digitalRead(pinA2);
  
  /* possibly wait before reading pin B, then read it        */
  debounce(0);
  int drB = digitalRead(pinB2);

  /* this updates the counter                                */
  if (drB == HIGH) {   /* low->high on B? */
  
    if (drA == HIGH) { /* check pin A */
    pos2++;  /* going clockwise: increment         */
    } else {
    pos2--;  /* going counterclockwise: decrement  */
    }
  
  } else {                       /* must be high to low on B */
  
    if (drA == LOW) {  /* check pin A */
    pos2++;  /* going clockwise: increment         */
    } else {
    pos2--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update */

} /* end ISR pin B Encoder 0  */




void TransIEEE754(float data, unsigned char *index1, unsigned char *index2, unsigned char *index3, unsigned char *index4)
{
  unsigned long temp = *(unsigned long *)&data;

  *index1 = temp >> 24;
  *index2 = (temp << 8) >> 24;
  *index3 = (temp <<16) >>24;
  *index4 = (temp <<24) >> 24;
}


float InverseTransIEEE754(unsigned char *data)
{
  unsigned char exponent, temp, fractionArray[3];
  double sign = 0.0, fractionTemp = 0.0, sumValue = 1.0;;
  float ret;

  if ((data[0] & 0x80) == 0x80)
  {
    sign = -1.0;
    exponent = (data[0] << 1) >> 1;
  }
  else
  {
    sign = 1.0;
    exponent = data[0];
  }

  exponent = exponent << 1;
  temp = data[1];

  if ((data[1] & 0x80) == 0x80) exponent = exponent | 0x01;

  temp = (temp << 1) >> 1;

  fractionArray[0] = temp;
  fractionArray[1] = data[2];
  fractionArray[2] = data[3];

  sumValue /= 2.0;
  if ((fractionArray[0] & 0x40) == 0x40) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x20) == 0x20) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x10) == 0x10) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x08) == 0x08) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x04) == 0x04) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x02) == 0x02) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[0] & 0x01) == 0x01) fractionTemp += sumValue;

  sumValue /= 2.0;
  if ((fractionArray[1] & 0x80) == 0x80) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x40) == 0x40) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x20) == 0x20) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x10) == 0x10) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x08) == 0x08) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x04) == 0x04) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x02) == 0x02) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[1] & 0x01) == 0x01) fractionTemp += sumValue;

  sumValue /= 2.0;
  if ((fractionArray[2] & 0x80) == 0x80) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x40) == 0x40) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x20) == 0x20) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x10) == 0x10) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x08) == 0x08) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x04) == 0x04) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x02) == 0x02) fractionTemp += sumValue;
  sumValue /= 2.0;
  if ((fractionArray[2] & 0x01) == 0x01) fractionTemp += sumValue;

  ret = (float)(sign * (1 + fractionTemp) * pow(2.0, (double)(exponent - 127)));

  return ret;
}


void encoder3(){
      unsigned char EncoderSendData[15];
      EncoderSendData[0] = 0x2A;
      EncoderSendData[13] = 0x0D;
      EncoderSendData[14] = 0x0A;
      
      
      TransIEEE754(pos0, &EncoderSendData[1], &EncoderSendData[2], &EncoderSendData[3], &EncoderSendData[4]);
      
      TransIEEE754(pos1, &EncoderSendData[5], &EncoderSendData[6], &EncoderSendData[7], &EncoderSendData[8]);
      
      TransIEEE754(pos2, &EncoderSendData[9], &EncoderSendData[10], &EncoderSendData[11], &EncoderSendData[12]);
      
      for (int i = 0; i < 15; i++){
        Serial.write(EncoderSendData[i]);
      }
      
      pos0 = 0;
      pos1 = 0;
      pos2 = 0;
      //delay(10);
}

  



void setup() {
  // initialize serial:
  Serial.begin(115200);
  
  // make the pins outputs:
  pinMode(pinA0, INPUT);
  pinMode(pinB0, INPUT);
  
  pinMode(pinA1, INPUT);
  pinMode(pinB1, INPUT);
  
  pinMode(pinA2, INPUT);
  pinMode(pinB2, INPUT);

  /* turn on pullup resistors                              */
  digitalWrite(pinA0, HIGH); 
  digitalWrite(pinB0, HIGH);

  digitalWrite(pinA1, HIGH); 
  digitalWrite(pinB1, HIGH);
  
  digitalWrite(pinA2, HIGH); 
  digitalWrite(pinB2, HIGH);
  
  //* attach interrupts 
  attachInterrupt(getIntNum(pinA0), irsPinAEn0, CHANGE);
  attachInterrupt(getIntNum(pinB0), isrPinBEn0, CHANGE);

  attachInterrupt(getIntNum(pinA1), irsPinAEn1, CHANGE);
  attachInterrupt(getIntNum(pinB1), isrPinBEn1, CHANGE);

  attachInterrupt(getIntNum(pinA2), irsPinAEn2, CHANGE);
  attachInterrupt(getIntNum(pinB2), isrPinBEn2, CHANGE);
        
  // signed for motor control pin
  pinMode(pinPWM0, OUTPUT);
  pinMode(pinDIR0, OUTPUT);

  pinMode(pinPWM1, OUTPUT);
  pinMode(pinDIR1, OUTPUT);

  pinMode(pinPWM2, OUTPUT);
  pinMode(pinDIR2, OUTPUT);
  
  
  Serial.println("Start");
}



void loop() { 
  unsigned char rc; 
  static int k = 0;
    
  if (Serial.available() > 0){ //=================================
     rc = Serial.read();
     
     if(rc == 'R'){
        encoder3();
     }
   else{ 
   
     if (rc == 0x3C){
            k = 0;
     }
     else if (rc == 0x3E){
        for (int i = 0; i < 4; i++) {
            duty0[i] = duty[i];
            duty1[i] = duty[i + 4];
            duty2[i] = duty[i + 8];
        }
        PWM0 = (long) InverseTransIEEE754((unsigned char*)duty0);
        PWM1 = (long) InverseTransIEEE754((unsigned char*)duty1);
        PWM2 = (long) InverseTransIEEE754((unsigned char*)duty2);
        
        analogWrite(pinPWM0, PWM0);
        
        analogWrite(pinPWM1, PWM1);
        
        analogWrite(pinPWM2, PWM2);
         
     }
     else{
        duty[k] = rc;
        k++;
     }
    }
  //============================================================== 
  } 
    
}

