int A,B;

void btData()
{
  while(myBT.available())
  {
        val = myBT.read();
        inReceive = true;
        bluetooth_data += val;
        Serial.println(bluetooth_data);
  }/*
  if(inReceive)
  {
        inReceive = false;
        //Serial.println(bluetooth_data);
        char buf[25];
        char* str;
        bluetooth_data.toCharArray(buf, sizeof(buf));
        Serial.println(String(buf));
        if((str = strtok(buf," ")) != NULL)
        {
           //Serial.println(String(str));
           A = String(str).toInt();
           if((str = strtok(NULL," ")) != NULL){
             //Serial.println(String(str));
             B = String(str).toInt();
           }
          Serial.println(String(A)+" "+String(B));
        }
        bluetooth_data = "";
    }*/
}
