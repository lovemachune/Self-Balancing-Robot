void btData()
{
    while(myBT.available())
    {
        //Serial.println(myBT.read());
        val = myBT.read();
        inReceive = true;
        bluetooth_data += val;
        
    }
    if(inReceive)
    {
        inReceive = false;
        //Serial.println(bluetooth_data);
        char buf[20];
        char* str;
        bluetooth_data.toCharArray(buf, sizeof(buf));
        Serial.println(String(buf));
        if((str = strtok(buf," ")) != NULL)
        {
           //Serial.println(String(str));
           tKP = String(str).toDouble();
           if((str = strtok(NULL," ")) != NULL){
             //Serial.println(String(str));
             tKI = String(str).toDouble();
           }
           if((str = strtok(NULL," ")) != NULL){
             //Serial.println(String(str));
             tKD = String(str).toDouble();
             can_set = true;
           }
        }
        if(can_set)
        {
          Serial.println(String(tKP) + " " + String(tKI) + " " + String(tKD)+ "hihi");
          motorA.SetControl(mode, reference,tKP,tKI,tKD);
          motorB.SetControl(mode, reference,tKP,tKI,tKD);
          can_set = false;
        }
        
        bluetooth_data = "";
    }
    //myBT.println( String(wheelAngleB)+","+String(phi));
}
