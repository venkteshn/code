  
#include <LiquidCrystal.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#define SET_bit(PORT,BIT)  PORT|=(1<<BIT)
#define CLR_bit(PORT,BIT)  PORT&=~(1<<BIT)
struct
{
  volatile unsigned int ISR1:1;
}FLAG;

const int pingPin = 7;//configuring as input pin for sensor

long readUltrasonic(int pin)

{

  SET_bit(DDRD,pin);

  CLR_bit(PORTD,pin);

  _delay_ms(2);

  SET_bit(PORTD,pin);

  _delay_ms(5);

  CLR_bit(PORTD,pin);

  CLR_bit(DDRD,pin);

  return pulseIn(pin, HIGH);

}
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

int main(void)
{
  long  inches, cm;
  Serial.begin(9600);
  SET_bit(DDRB,PB0);
  CLR_bit(DDRD,PD7);//  sensor
  CLR_bit(DDRD,PD2);//ignition switch
  

  EICRA|=(1<<ISC00);
  EICRA&=~(1<<ISC01);// Any logical change will raise interrupt
  EIMSK|=(1<<INT0);//Local Interrupt enable
  sei();//
  
while(1)
{
if(FLAG.ISR1==1)
{
  cm = microsecondsToCentimeters(readUltrasonic(7));
if(PIND & (1<<PD2))
          {
            CLR_bit(PORTB,PB0);
          }
else if(PIND & ~(1<<PD2))
{

  if(cm<250)//buzzer range
  {
    SET_bit(PORTB,PB0);
  }
  
  else
  {
    CLR_bit(PORTB,PB0);
  }
  
}
  
          
 }
}
          }
ISR(INT0_vect)  //interrupt for switch 
            {
              cli();
            FLAG.ISR1=1;
              sei(); 
            }
