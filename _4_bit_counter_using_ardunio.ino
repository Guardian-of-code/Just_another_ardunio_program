//coded by Simha
int i=0; 
int a =13;
int b =12;
int c =11;
int d =10;
//initalize setup
void setup() {
  Serial.begin(9600);
 pinMode(a,OUTPUT);
 pinMode(b,OUTPUT);
 pinMode(c,OUTPUT);
 pinMode(d,OUTPUT);

}

void loop() {
  //start of void loop
  
  for(i=0;i<16;i++){
 
    digitalWrite(a,bool(i/8==1));
    digitalWrite(b,bool((i%8)/4==1));
    digitalWrite(c,bool((i/2)%2==1));
    digitalWrite(d,bool((i%2==1)));

     delay(1000);}
  

}
