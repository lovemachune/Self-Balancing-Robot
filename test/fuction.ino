void data_sent()
{
    myBT.print(currentAngleA);
    myBT.print(" ");
    myBT.print(currentAngleB);
    myBT.print(" ");
    //myBT.print("Car Angle :");
    myBT.println(kalAngleX);
}

void move(char key)
{
  switch(key)
  {
    case 'w':
      motorA.move(50,1);
      motorB.move(50,1);
      break;
    case 'a':
      motorA.move(85,1);
      motorB.move(50,1);
      break;
    case 's':
      motorA.move(50,0);
      motorB.move(50,0);
      break;
    case 'd':
      motorA.move(50,1);
      motorB.move(85,1);
      break;
    case 'x':
      motorA.stop();
      motorB.stop();
      break; 
  }
}
