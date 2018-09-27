#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned int  flag=0;//GLOBAL VARIABLE DECLERATION
//int adc_init1();//ADC FUNCTION DECLERATION
unsigned int adc1=0;
unsigned int ADC12;
void defogger()
{
  
   adc1= adcread();
      Serial.println(adc1);//PRINTING VALUE OF ADC ON SERIAL MONITOR
     
     
          if((adc1>=0)&&(adc1<=150))//0 DEGREE 
          {
            PORTC|=(1<<PC3);
            //OCR2B=250;//DEFOGGER ON
            //Serial.println("Defogger ON,Bulb intensity high");
          }
         
          else
          {
            PORTC&=~(1<<PC3);
          }
  
}

int main()
{
  DDRC &=~(1<<PC2);//INPUT TEMPERATURE SENSOR PIN A1
  DDRC |=(1<<PC3);//OUTPUT BULB AND MULTIMETER PIN A0
  
  DDRD &=~(1<<PD0);
 
  sei();//ENABEL INTERRUPT

  PCICR|=(1<<PCIE2);
  PCMSK2|=(1<<PCINT17);
  unsigned int adc1=0;
  acdinit(1);
  while(1)
  {
   
    if(flag==1)
    {
      //_delay_ms(2000);//DELAY OF 2 SECOND
      defogger();
     
    }
    else
    {
      PORTC&=~(1<<PC3);//DEFOGGER OFF
    }
  }
  }


ISR(PCINT2_vect) //INTERRUPT FUNCTION
{
 flag=!flag;//TOGGLEING THE VALUE OF FLAG
}

void acdinit(int flag2)
{
  ADCSRA |=(1<<ADEN);
  if(flag2 == 1)
  {
   
  ADMUX|=(1<<REFS0);
  }
  else
  {
    ADMUX &=~(1<<REFS0);
  }
}

  

int adcread()
{
  ADMUX |=0b01000010;
  ADCSRA|=(1<<ADSC);
  while((ADCSRA) & (1<<ADSC));
  return ADC12;
}




