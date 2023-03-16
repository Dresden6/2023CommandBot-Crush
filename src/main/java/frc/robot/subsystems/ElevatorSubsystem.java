package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    CANSparkMax controllerLeft;
    CANSparkMax controllerRight;

    // Using differential drive for this because it gives us a few goodies for free,
    // like squaring the inputs, deadband and maximum output.
    private DifferentialDrive elevatorDrive;
    private RelativeEncoder encoder;
    
    private final double minHeight = 0.0;
    private final double maxHeight = 1.143; // This is the highest it can possibly go, about 45 inches. We might want to change this to a smaller value for safety's sake?

    public void init()
    {
        controllerLeft = new CANSparkMax(7, MotorType.kBrushless);
        controllerRight = new CANSparkMax(8, MotorType.kBrushless);
        controllerRight.setInverted(true);

        elevatorDrive = new DifferentialDrive(controllerLeft, controllerRight);

        encoder = controllerLeft.getEncoder();

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
        encoder.setPositionConversionFactor((0.02794 * Math.PI) / 27.0);
    }

    public boolean isAtTop()
    {
        var position = encoder.getPosition();
        return position >= maxHeight; // TODO: Might want to add a fudge factor here for safety?
    }

    public boolean isAtBottom()
    {
        var position = encoder.getPosition();
        return position <= minHeight; // TODO: Might want to add a fudge factor here for safety?
    }

    public void move(double speed)
    {
        // Check if we are already at the top or bottom and stop moving if so.
        if (speed > 0 && isAtTop())
        {
            speed = 0;
        }
        else if (speed < 0 && isAtBottom())
        {
            speed = 0;
        }

        elevatorDrive.arcadeDrive(speed, 0);
    }

    public void resetEncoders()
    {
        encoder.setPosition(0);
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
