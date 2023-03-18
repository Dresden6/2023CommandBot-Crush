package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private RelativeEncoder encoder;
    public void init()
    {
        // Setup motor
        intakeMotor = new CANSparkMax(10, MotorType.kBrushless);

        // Setup encoder
        encoder = intakeMotor.getEncoder();
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
