#include<avr/io.h>

#include <avr/interrupt.h>

#include<util/delay.h>

#define SET_BIT(PORT,BIT) PORT |=(1<<BIT)

#define CLR_BIT(PORT,BIT) PORT &= ~(1<<BIT)

volatile unsigned int rain_digital=0;

struct

{

volatile unsigned int ISR1:1; // Switch up interrupt

volatile unsigned int ISR2:1; // Switch down interrupt

}FLAG;

 

int main()

{

  SET_BIT(DDRD,DDD4);     //Digital pin 4 as input to motor driver

  SET_BIT(DDRD,DDD5);     //Digital pin 5 as input to motor driver

  CLR_BIT(PORTD,PORTD4);  

  CLR_BIT(PORTD,PORTD5);

  CLR_BIT(DDRD,DDD2);     //Input switch to move window up

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

 

  Serial.begin(9600);
 

while(1)

  {
    //DC motor doesnot rotate
    
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

 

}

 




ISR(INT0_vect)
{
FLAG.ISR1=!FLAG.ISR1;
}

ISR(INT1_vect)
{
FLAG.ISR2=!FLAG.ISR2;
}


