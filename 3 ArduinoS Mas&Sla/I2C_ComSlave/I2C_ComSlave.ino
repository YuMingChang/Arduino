char str[12];

void setup()
{
  Serial.begin(115200);
}

void loop() {
  
   for(int i=0; i<sizeof(str); i++){
    str[i] = random(48, 57);
    Serial.write(str[i]);
    if( i == sizeof(str)-1 ) Serial.println();
   }

  delay(100);
}
