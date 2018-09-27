#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define SET_BIT(PORT,BIT) PORT |=(1<<BIT)

#define CLR_BIT(PORT,BIT) PORT &= ~(1<<BIT)

unsigned int adc=0;
unsigned int  flag=0;//GLOBAL VARIABLE DECLERATION
int adc_init();//ADC FUNCTION DECLERATION
unsigned int ADC1;
int adc_init1();//ADC FUNCTION DECLERATION
unsigned int adc1=0;

struct

{

volatile unsigned int ISR1:1; // Switch up interrupt

volatile unsigned int ISR2:1; // Switch down interrupt

}FLAG;


long readUltrasonic(int pin)

{

  SET_BIT(DDRD,pin);

  CLR_BIT(PORTD,pin);

  _delay_ms(2);

  SET_BIT(PORTD,pin);

  _delay_ms(5);

  CLR_BIT(PORTD,pin);

  CLR_BIT(DDRD,pin);

  return pulseIn(pin, HIGH);

}
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


 void wiper()
  {
   setup();
     adc= adc_init();
      Serial.println(adc);//PRINTING VALUE OF ADC ON SERIAL MONITOR
     
     
          if(adc<=100)
          {
            
            OCR0B=0;//WIPER OFF
            Serial.println("Wiper OFF,motor speed zero,rain intensity zero");
          }
          else if((adc>100)&&(adc<=500))
          {
            
             OCR0B=250;//WIPER LOW
            Serial.println("DC motor at low rpm,rain intensity is low");

          }
          else if((adc>500)&&(adc<=800))
          {
            
            
            OCR0B=250;//WIPER MEDIUM
            Serial.println("DC motor at medium rpm,rain intensity is medium");     
          }
          else if(adc>800)
          {
            OCR0B=250;//WIPER HIGH
            Serial.println("dc motor at high rpm,rain intensity is high");
            Serial.println("Rain blower on");
          }
   else
   {
      OCR0B=0;//WIPER OFF
   }
  
    }
  
    
 void defogger()
{
    acdinit(1);
   adc1= adcread();
      Serial.println(adc1);//PRINTING VALUE OF ADC ON SERIAL MONITOR
     
     
          if((adc1>=0)&&(adc1<=150))//0 DEGREE 
          {
            PORTB|=(1<<PB1);
            PORTC|=(1<<PC3);
            //OCR2B=250;//DEFOGGER ON
            //Serial.println("Defogger ON,Bulb intensity high");
          }
    else
    {
      PORTC&=~(1<<PC3);//DEFOGGER OFF
      PORTB&=~(1<<PB1);
    }
  
}
   
  void anti_theft()
{
  
   long  inches, cm;
   cm = microsecondsToCentimeters(readUltrasonic(7));
   	   if(PIND & (1<<PD2))
          {
            CLR_BIT(PORTB,PB0);
          }
       else if(PIND & ~(1<<PD2))
       {
		  if(cm<250)//buzzer range
 		 {
 		   SET_BIT(PORTB,PB0);
 		 }
         else
 		 {
    		CLR_BIT(PORTB,PB0);
  		 }
 	   }
}

void window()
{
   if((FLAG.ISR1==0) && (FLAG.ISR2==0)) 

    {

      CLR_BIT(PORTD,PORTD4);     

      CLR_BIT(PORTD,PORTD5);

    }

    //DC motor rotates clockwise, window moves upward

    else if ((FLAG.ISR1==1) && (FLAG.ISR2==0))

    {

      PORTD |=(1<<PD5);                   

    }
 //DC motor rotates ant-clockwise, window moves downward

else if((FLAG.ISR1==0) && (FLAG.ISR2==1))

      {

        PORTD |=(1<<PD4);

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
           
  DDRB |=(1<<PB1);
  DDRC &=~(1<<PC2);//INPUT TEMPERATURE SENSOR PIN A1
  DDRC |=(1<<PC3);//OUTPUT BULB AND MULTIMETER PIN A0        
  //sei();//ENABEL INTERRUPT
  
  long  inches, cm;
  Serial.begin(9600);
  SET_BIT(DDRB,PB0);
  CLR_BIT(DDRD,PD7);//  sensor
 
  unsigned int adc1=0;
//  acdinit(1);
  
  SET_BIT(DDRD,DDD4);     //Digital pin 4 as input to motor driver

  SET_BIT(DDRD,DDD5);     //Digital pin 5 as input to motor driver

  CLR_BIT(PORTD,PORTD4);  

  CLR_BIT(PORTD,PORTD5);

  CLR_BIT(DDRD,DDD1);     //Input switch to move window up

  CLR_BIT(DDRD,DDD3);     //Input switch to move window down

 

  //Interrupt 0
  EICRA &=~(1<<ISC01);

  EICRA |=(1<<ISC00);

  EIMSK|=(1<<INT0); //Enable interrupt 0


 //Interrupt 1

  EICRA &=~(1<<ISC11);

  EICRA |=(1<<ISC10);

  EIMSK|=(1<<INT1); //Enable interrupt 1


  SREG |=(1<<7);   //Enable global interrupts
  
  
           
           
  while(1)
  {
   
    if(flag==1)
    {
       wiper();
      window();
      
    }
    else
    {
   defogger();
     anti_theft();
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
  

void setup()
{
 /* ADCSRA |= (1<<ADEN);	//ADC Initialisation
  ADMUX |= (1<<REFS0);	//set reference voltage
  /* control and status register prescaling 
  ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));*/
    ADMUX |=(1<<REFS0)|(1<<MUX0);//INPUT AT ADC1 AND AVCC with external capacitor at AREF pin
  ADCSRA|=(1<<ADEN);//ENABLE ADC
 ADCSRA|=(1<<ADSC);//START OF COVERSION ON
}

int adc_init()//ADC FUNCTION FOR POTENTIOMETER DIGITAL VALUE
{
   ADMUX |=(1<<MUX0);
 // ADMUX |=(1<<REFS0)|(1<<MUX0);//INPUT AT ADC1 AND AVCC with external capacitor at AREF pin
 // ADCSRA|=(1<<ADEN);//ENABLE ADC
// ADCSRA|=(1<<ADSC);//START OF COVERSION ON
  while(ADSC==1);//CHECKING END OF CONVERSION
  return(ADC);//STORING DIGITAL VALUE
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
  return ADC;
}


ISR(INT0_vect)
{
FLAG.ISR1=!FLAG.ISR1;
}

ISR(INT1_vect)
{
FLAG.ISR2=!FLAG.ISR2;
}

