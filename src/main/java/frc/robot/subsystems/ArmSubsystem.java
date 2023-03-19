package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private RelativeEncoder encoder;
    private PIDController pidController;

    // TODO: Need to set the minimum and maximum allowed angles for the arm
    private final double minAngle = 0;
    private final double maxAngle = 70; // Could possibly be 80? I'll just say 60 for now

    public void init() {
        // Setup motor
        armMotor = new CANSparkMax(armMotorPort, MotorType.kBrushless);

        // Setup encoder
        encoder = armMotor.getEncoder();
        // Pitch diameter is 64/16 inches
        // Gear ratio is 1:125
        // Since we only care about radians of rotation I don't think
        // we need to factor the pitch diameter into the equation?
        encoder.setPositionConversionFactor(1.0 / 125.0);
        encoder.setPosition(0);

        pidController = new PIDController(armKP, armKI, armKD);
        pidController.setTolerance(armPIDTolerance);

    }

    /**
     * Set the arm to a specific position. Must be called periodically
     * @param position in degrees
     */
    public void setArmPosition(double position) {
        double pidCalc = pidController.calculate(getPosition(), position / 360); // Convert to rotations
        setArmVolts(pidCalc + armKF); // Arbitrary FeedForward. I think this is what 1339 is doing for our wrist, but
                                      // it might cause some weird behavior because KF is calculated at the point
                                      // where the motor has to work the hardest, when the wrist is straight up it
                                      // could be weird because it's just turing it in one direction constantly. It's
                                      // probably fine as long as KF isn't too high.
    }

    public void setArmVolts(double volts) {
        armMotor.setVoltage(volts);
    }

    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }

    public void disable() {
        armMotor.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
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
    public double getDegrees() {
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
        } else if (speed > 0 && getDegrees() >= maxAngle) {
            speed = 0;
        }
    }

    public void resetEncoder(double position) {
        encoder.setPosition(position);
    }
    public void resetEncoder() {
        resetEncoder(0);
    }


}