package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax leftElevatorMotor;
    private CANSparkMax rightElevatorMotor;

    private RelativeEncoder encoder;

    private Constraints elevatorProfileConstraints;
    private ProfiledPIDController elevatorPIDController;

    private double lastSpeed = 0;
    private double lastTime = Timer.getFPGATimestamp(); 
    
    private final double minHeight = 0.0;
    private final double maxHeight = -2; // This is the highest it can possibly go, about 45 inches. We might want to change this to a smaller value for safety's sake?

    // ElevatorFeedforward feedforward = new ElevatorFeedforward(elevatorKS, elevatorKG, elevatorKV);

    public void init() {

        elevatorProfileConstraints = new Constraints(elevatorMaxVel, elevatorMaxAccel);
        elevatorPIDController = new ProfiledPIDController(elevatorKP, elevatorKI, elevatorKD, elevatorProfileConstraints);

        elevatorPIDController.setTolerance(elevatorPIDTolerance);

        leftElevatorMotor = new CANSparkMax(7, MotorType.kBrushless);
        rightElevatorMotor = new CANSparkMax(8, MotorType.kBrushless);
        rightElevatorMotor.follow(leftElevatorMotor, true);

 
        


        encoder = leftElevatorMotor.getEncoder();

        // When the match starts we should assume that the elevator is either at the
        // minimum or the maximum position and reset the encoder accordingly. 
        encoder.setPosition(0);

        // We need to set the position conversion factor which says how many meters
        // the elevator moves with one full rotation of the motor. To do this, we must
        // know the gear ratio and the diameter of the gear/wheel driving the elevator.
        /*
         * gear ratio: 1:27
         * gear diameter: 22 teeth / 20 for diametral pitch? 
         * I am unsure if this is what we're looking for
         */
        encoder.setPositionConversionFactor((1 * Math.PI) / 27.0);
    }
    
    @Override 
    public void periodic()
    {
        // Log dashboard values
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }


  
    /**
     * Set the goal for the elevator PID controller. This must be called periodically.
     * @param goal
     */
    public void setPosition(double positionGoal) {
        
        double pidVal = elevatorPIDController.calculate(getPosition(), positionGoal);
        double acceleration = (elevatorPIDController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

        double motorVoltsOutput = pidVal + elevatorKF; // Arbitrary FeedForward


        // TODO Check signs on these
        // Motor safety. Might want to add limit switches at some point.
        if(motorVoltsOutput > 0 && isAtTop()){

            motorVoltsOutput = 0;
        }
        else if(motorVoltsOutput < 0 && isAtBottom()){

            motorVoltsOutput = 0;
        } else {

            setElevatorVolts(motorVoltsOutput);
        }

        lastSpeed = elevatorPIDController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }


    public double getPositionError() {
        return elevatorPIDController.getPositionError();
    }
    
    public boolean isAtGoal() {
        return elevatorPIDController.atGoal();
    }

    public void setElevatorVolts(double volts) {
        leftElevatorMotor.setVoltage(volts);
    }

    public void disable() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    public boolean isAtTop()
    {
        var position = encoder.getPosition();
        return position <= maxHeight; // TODO: Might want to add a fudge factor here for safety?
        
        //SmartDashboard.putData("elevator", elevatorDrive);
    }

    public boolean isAtBottom()
    {
        var position = encoder.getPosition();
        return position >= minHeight; // TODO: Might want to add a fudge factor here for safety?
    }

    public void resetEncoder(double position) {
        encoder.setPosition(position);
    }
    public void resetEncoder() {
        resetEncoder(0);
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    public double getMaxPosition()
    {
        return maxHeight;
    }
}
