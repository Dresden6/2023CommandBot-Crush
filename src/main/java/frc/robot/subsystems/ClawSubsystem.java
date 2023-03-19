package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClawConstants.*;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax gripMotor;
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;

    // placeholders for now
    // TODO: get the real values of these !!(originally 0 and 1)
    private final double minAngle = -0.1;
    private final double maxAngle = 0.6;


    public void init()
    {
        // Setup motor
        gripMotor = new CANSparkMax(gripMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);

        // Setup encoder
        encoder = gripMotor.getEncoder();
        encoder.setPositionConversionFactor(1.0/25.0);
        encoder.setPosition(0);
    }
    
    @Override 
    public void periodic()
    {
        // Log dashboard values
        SmartDashboard.putNumber("Claw Position", encoder.getPosition());
    }

    public double getDegrees()
    {
        // Position is returning number of rotations.
        // Multiply by 360 to get degrees. (One full rotation is 360 degrees)
        return encoder.getPosition() * 360;
    }

    /**
     * Run at percent [-1, 1]
     * @param speed
     */
    public void runIntake(double speed)
    {
        intakeMotor.set(speed);
    }

    public void disable() {
        gripMotor.set(0);
        intakeMotor.set(0);
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

    public void moveClaw(double speed)
    {
        if (speed > 0 && isOpen())
        {
            speed = 0;
        }
        else if (speed < 0 && isClosed())
        {
            speed = 0; 
        }

        gripMotor.set(speed);
    }

    public void resetEncoders()
    {
        encoder.setPosition(0);
    }
}
