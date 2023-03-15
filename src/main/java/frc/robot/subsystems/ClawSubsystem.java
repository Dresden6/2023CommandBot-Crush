package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase{
    private CANSparkMax clawMotorLeft;
    private CANSparkMax ClawMotorRight;
    
    private DifferentialDrive clawDrive;
    private RelativeEncoder encoder;

    // placeholders for now
    // TODO: get the real values of these !!
    private final double minAngle = 0;
    private final double maxAngle = 1;


    public void init()
    {
        // Setup motor
        clawMotorLeft = new CANSparkMax(10, MotorType.kBrushless);
        ClawMotorRight = new CANSparkMax(11, MotorType.kBrushless);
        ClawMotorRight.setInverted(true);

        // Setup encoder
        encoder = clawMotorLeft.getEncoder();
        // TODO: get the conversion factor for this, i assume there is one
        //encoder.setPositionConversionFactor();
        encoder.setPosition(0);
    }

    public double getDegrees()
    {
        // Position is returning number of rotations.
        // Multiply by 360 to get degrees. (One full rotation is 360 degrees)
        return encoder.getPosition() * 360;
    }

    public boolean isOpen()
    {
        var degrees = getDegrees();
        return degrees >= maxAngle; 
    }

    public boolean isClosed()
    {
        var degrees = getDegrees();
        return degrees <= minAngle;
    }

    public void move(double speed)
    {
        if (speed > 0 && isOpen())
        {
            speed = 0;
        }
        else if (speed < 0 && isClosed())
        {
            speed = 0; 
        }

        clawDrive.arcadeDrive(speed, 0);
    }

    public void resetEncoders()
    {
        encoder.setPosition(0);
    }
}
