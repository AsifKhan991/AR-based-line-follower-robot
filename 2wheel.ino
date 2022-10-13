int left=0;
int right=0;
bool dirL=1;
bool dirR=1;

void setup() {
  Serial.begin(57600);
  pinMode(3,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(11,OUTPUT);
}

void get_data(){
    int arr[3]={0,0,0}; //index 0 is for direction, 1 is left speed and 2 is right speed
    
  /*index 0 value 0 means both forward
    index 0 value 01 means left forward right reverse
    index 0 value 16 means left reverse right forward
    index 0 value 17 means both reverse*/
    
    if(Serial.available()==3){
     arr[0]=Serial.read();
     arr[1]=Serial.read();
     arr[2]=Serial.read();
    
      byte dir=arr[0];
      if(dir==0){
        dirL=1;
        dirR=1;
      }else if(dir==17){
        dirL=0;
        dirR=0;
      }
      else if(dir==1){
        dirL=0;
        dirR=1;
      }
      else if(dir==16){
        dirL=1;
        dirR=0;
      }
      else{
        return;
      }
    left=arr[1];
    right=arr[2];
    }
}

void loop() {
  get_data();

  if(dirL){
    analogWrite(3,left);
    analogWrite(5,0);
  }else{
    analogWrite(3,0);
    analogWrite(5,left);
  }

  if(dirR){
    analogWrite(6,right);
    analogWrite(11,0);
  }else{
    analogWrite(6,0);
    analogWrite(11,right);
  }
}
