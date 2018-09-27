#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
unsigned int adc=0;
unsigned int  flag=0;//GLOBAL VARIABLE DECLERATION
int adc_init();//ADC FUNCTION DECLERATION

 void wiper()
  {
     adc= adc_init();
      Serial.println(adc);//PRINTING VALUE OF ADC ON SERIAL MONITOR
     
     
          if(adc<=100)
          {
            
            OCR0B=0;//WIPER OFF
            Serial.println("Wiper OFF,motor speed zero,rain intensity zero");
          }
          else if((adc>100)&&(adc<=500))
          {
            
             OCR0B=50;//WIPER LOW
            Serial.println("DC motor at low rpm,rain intensity is low");

          }
          else if((adc>500)&&(adc<=800))
          {
            
            
            OCR0B=150;//WIPER MEDIUM
            Serial.println("DC motor at medium rpm,rain intensity is medium");     
          }
          else
          {
            OCR0B=250;//WIPER HIGH
            Serial.println("dc motor at high rpm,rain intensity is high");
            Serial.println("Rain blower on");
          }
    }
  
    
    
  


int main()
{
  Serial.begin(9600);//SERIAL MONITOR
 
  DDRC &=~(1<<PC1);//INPUT POTENTIOMETER PIN A1
  DDRD &=~(1<<PD0);//INPUT SWITCH PIN 12
  DDRC |=(1<<PC0);//OUTPUT MOTOR
   
 
  sei();//ENABELING INTERRUPT
  PCICR|=(1<<PCIE2);
  PCMSK2|=(1<<PCINT17);
  
  OCR0A=255;//STORING MAX VALUE FOR PWM
  TCCR0A&=~((1<<WGM00));//FOR CTC MODE
  TCCR0A|=(1<<WGM01);//FOR CTC MODE
  TCCR0B&=~(1<<WGM02);//FOR CTC MODE
  TCCR0B|=(1<<CS00)|(1<<CS02);//CLOCK 1024 PRESCALAR
  
  TIMSK0|=(1<<OCIE0A)|(1<<OCIE0B);//ENABLING CTC
  
  unsigned int adc=0;
 
  while(1)
  {
   
    if(flag==1)
    {
      //_delay_ms(2000);//DELAY OF 2 SECOND
      wiper();
     
    }
  }
}


ISR(PCINT2_vect) //INTERRUPT FUNCTION
{
 flag=!flag;//TOGGLEING THE VALUE OF FLAG
}


ISR(TIMER0_COMPA_vect)//TIMER COMPARE A FUNCTION
{
  PORTC|=(1<<PC0);//IT WILL COMPARE WITH OCR0A=255 AND WILL GIVE HIGH OUTPUT
}


ISR(TIMER0_COMPB_vect)//TIMER COMPARE B FUNCTION
{
  PORTC&=~(1<<PC0);//IT WILL COMPARE WITH OCR0B VALUES AND WILL GIVE LOW OUTPUT TILL THAT VALUE
}
  

int adc_init()//ADC FUNCTION FOR POTENTIOMETER DIGITAL VALUE
{
  ADMUX |=(1<<REFS0)|(1<<MUX0);//INPUT AT ADC1 AND AVCC with external capacitor at AREF pin
  ADCSRA|=(1<<ADEN);//ENABLE ADC
 ADCSRA|=(1<<ADSC);//START OF COVERSION ON
  while(ADSC==1);//CHECKING END OF CONVERSION
  return(ADC);//STORING DIGITAL VALUE
}


