void data_sent()
{
    myBT.print("AngleA: "); 
    myBT.print(currentAngleA);
    myBT.print(" AngleB: ");
    myBT.println(currentAngleB);
    myBT.print("SpeedA: ");
    myBT.print(speedA);
    myBT.print(" SpeedB: ");
    myBT.println(speedB);
    myBT.println("Car Angle :");
    myBT.print(angle_pitch_output);
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