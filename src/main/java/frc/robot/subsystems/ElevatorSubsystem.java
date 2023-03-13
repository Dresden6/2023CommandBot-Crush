package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Elevator motor controller group
    private MotorControllerGroup motorGroup;
    private RelativeEncoder encoder;
    

    public void init()
    {
        var controllerLeft = new CANSparkMax(0, MotorType.kBrushless);
        var controllerRight = new CANSparkMax(1, MotorType.kBrushless);
        motorGroup = new MotorControllerGroup(controllerLeft, controllerRight);
        encoder = controllerLeft.getEncoder();

        // When the match starts we should assume that the elevator is either at the
        // minimum or the maximum position and reset the encoder accordingly. 
        encoder.setPosition(0);

        // We need to set the position conversion factor which says how many meters
        // the elevator moves with one full rotation of the motor. To do this, we must
        // know the gear ratio and the diameter of the gear/wheel driving the elevator.
        encoder.setPositionConversionFactor(1);
    }

    public boolean isAtTop(int pos)
    {
        // PLACEHOLDER; we can put a real value here later
        int top = 1;

        if(pos == top)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean isAtBottom(int pos)
    {
        // PLACEHOLDER; we can put a real value here later
        int bottom = 0;

        if(pos == bottom)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void move(double amt)
    {
        motorGroup.set(amt);
    }
}
