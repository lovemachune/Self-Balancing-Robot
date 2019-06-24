int A;
double B;

void btData()
{
  myBT.println("");
  while(myBT.available())
  {
        val = myBT.read();
        inReceive = true;
        bluetooth_data += val;
  }
  if(inReceive)
  {
        inReceive = false;
        //Serial.println(bluetooth_data);
        char buf[25];
        char* str;
        bluetooth_data.toCharArray(buf, sizeof(buf));
        //Serial.println(String(buf));
        if((str = strtok(buf," ")) != NULL)
        {
           A = String(str).toInt();
           if((str = strtok(NULL," ")) != NULL){
             B = String(str).toDouble();
             can_set = true;
           }
          Serial.println(String(A)+" "+String(B));
        }
        if(can_set)
        {
          motorA.SetDefaultPWM(A/2);
          motorB.SetDefaultPWM(-A/2);
          motorA.angleController.SetReference(B*3/10000+0.018*0.3);
          motorB.angleController.SetReference(B*3/10000+0.018*0.3);
        }
        bluetooth_data = "";
    }
}
void R_PI_data()
{
  while(!Serial.available()){}
  if(!START){
    Serial.println("start");
    START = true;
  }
  while(Serial.available())
  {
    val = Serial.read();
    inReceive = true;
    bluetooth_data += val;
  }
  Serial.println(bluetooth_data);
  if(inReceive && !RUNNING)
  {
      if(bluetooth_data == "t")
      {
        reference = 0.018;
        motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorA.SetDefaultPWM(-20);
        motorB.SetDefaultPWM(20);
        STRAIT = false;
        RUNNING = true;
      }
      if(bluetooth_data == "r")
      {
        reference = 0.018;
        motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorA.SetDefaultPWM(20);
        motorB.SetDefaultPWM(-20);
        STRAIT = false;
        RUNNING = true;
      }
      if(bluetooth_data == "c")
      {
        END=true;
        reference = 0.018;
        motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        motorA.SetDefaultPWM(0);
        motorB.SetDefaultPWM(0);
      }
      inReceive = false;
      bluetooth_data = ""; 
  }
}
