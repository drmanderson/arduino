int clock= 12;//IC pin 11 (yellow)
int latch=8;//IC pin 12 (green) 
int data=11;//IC pin 14 (blue)

byte pattern[]=
{   
B00000001,   
B00000010,   
B00000100,   
B00001000,   
B00010000,   
B00100000,   
B01000000,   
B10000000,   
B01000000,   
B00100000,   
B00010000,   
B00001000,   
B00000100,   
B00000010 
}; 
int index =0;
int count=sizeof(pattern);
void setup() 
{   
pinMode(clock,OUTPUT);   
pinMode(latch,OUTPUT);   
pinMode(data,OUTPUT); 
} 
void loop() 
{   
digitalWrite(latch,LOW);   
shiftOut(data,clock,MSBFIRST,pattern[index]);  
digitalWrite(latch,HIGH);   
delay(150);   
index++;   
if(index>=count)     
index=0; 
}
