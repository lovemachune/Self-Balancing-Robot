void btData()
{
    while(myBT.available())
    {
        inReceive = true;
        val = myBT.read();
        bluetooth_data += val; 
    }
    if(inReceive)
    {
        inReceive = false;
        Serial.println(bluetooth_data);
        bluetooth_data = "";
    }
}
