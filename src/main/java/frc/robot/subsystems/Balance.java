package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

public class Balance extends DriveSubsystem{
    private AHRS ahrs;

    // sets the roll value from the gyroscope to a variable
    float roll = ahrs.getRoll();

    /*
     * if the roll value is outside of a certain range, the robot will go
     * forwards or backwards to automatically balance itself
     * 
     * if the roll value is outside of that range, it will stop
     */
    if(roll > 5 || roll < -5)
    {
      if(roll > 5)
      {
        motorFrontLeft.set(ControlMode.PercentOutput, -0.2);
        motorFrontRight.set(ControlMode.PercentOutput, 0.2);
      }
      if(roll < -5)
      {
        motorFrontLeft.set(ControlMode.PercentOutput, 0.2);
        motorFrontRight.set(ControlMode.PercentOutput, -0.2);
      }
    }
    else
    {
        motorFrontLeft.set(ControlMode.PercentOutput, 0);
      motorFrontRight.set(ControlMode.PercentOutput, 0);
    }
}

