package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private RelativeEncoder encoder;

    // TODO: Need to set the minimum and maximum allowed angles for the arm
    private final double minAngle = -30;
    private final double maxAngle = 60; // Could possibly be 80? I'll just say 60 for now

    public void init()
    {
        // Setup motor
        armMotor = new CANSparkMax(9, MotorType.kBrushless);

        // Setup encoder 
        encoder = armMotor.getEncoder();
        encoder.setPositionConversionFactor(1.0 / 80.0); 
        encoder.setPosition(0);
    }

    /*
     * Return degrees arm is currently at.
     */
    public double getDegrees()
    {
        // Position is returning number of rotations.
        // Multiply by 360 to get degrees. (One full rotation is 360 degrees)
        return encoder.getPosition() * 360;
    }

    public void swing(double speed)
    {
        if (speed < 0 && getDegrees() <= minAngle)
        {
            speed = 0;
        }
        else if (speed > 0 && getDegrees() >= maxAngle)
        {
            speed = 0;
        }

        armMotor.set(speed);
    }
}