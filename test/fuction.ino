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
      motorA.Rotate(50);
      motorB.Rotate(50);
      break;
    case 'a':
      motorA.Rotate(85);
      motorB.Rotate(50);
      break;
    case 's':
      motorA.Rotate(-50);
      motorB.Rotate(-50);
      break;
    case 'd':
      motorA.Rotate(50);
      motorB.Rotate(85);
      break;
    case 'x':
      motorA.Brake();
      motorB.Brake();
      break; 
  }
}
