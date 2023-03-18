package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private RelativeEncoder encoder;

    // TODO: Need to set the minimum and maximum allowed angles for the arm
    private final double minAngle = 0;
    private final double maxAngle = 70; // Could possibly be 80? I'll just say 60 for now

    public void init()
    {
        // Setup motor
        armMotor = new CANSparkMax(9, MotorType.kBrushless);

        // Setup encoder 
        encoder = armMotor.getEncoder();
        // Pitch diameter is 64/16 inches
        // Gear ratio is 1:125
        // Since we only care about radians of rotation I don't think 
        // we need to factor the pitch diameter into the equation?
        encoder.setPositionConversionFactor(0.25 / 125.0); 
        encoder.setPosition(0);
    }

    @Override 
    public void periodic()
    {
        // Log dashboard values
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
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

        armMotor.set(speed);
        if (speed < 0 && getDegrees() <= minAngle)
        {
            speed = 0;
        }
        else if (speed > 0 && getDegrees() >= maxAngle)
        {
            speed = 0;
        }

        
    }
}