package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;
    
    public void init()
    {
        // Setup motor
        intakeMotor = new CANSparkMax(10, MotorType.kBrushless);

        // Setup encoder
    }

    public void activate()
    {
        intakeMotor.set(0.5);
    }

    public void deactivate()
    {
        intakeMotor.set(0);
    }
}
