package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetMoveElevatorCommand extends CommandBase {
    
    final XboxController controller;
    final ElevatorSubsystem elevatorSubsystem; 
    final MovePosition position;

    final double top;
    final double middle;
    final double bottom;

    private double target;
    private double elevatorPosition;

    public static enum MovePosition {
        Top, 
        Middle, 
        Bottom
    }

    /**
     * Move elevator to set point.
     * @param controller
     * @param subsystem
     * @param position
     */
    public SetMoveElevatorCommand(XboxController controller, ElevatorSubsystem subsystem, MovePosition position)
    {
        this.controller = controller;
        this.elevatorSubsystem = subsystem;
        this.position = position;
        this.addRequirements(elevatorSubsystem);
        this.top = elevatorSubsystem.getMaxPosition();
        this.middle = top/2.0;
        this.bottom = 0;
    }

    @Override 
    public void initialize()
    {
        switch (this.position) {
            case Top:
                target = top;
                break;
            case Middle:
                target = middle;
                break;
            case Bottom:
                target = bottom;
                break;
            default:
                break;
        }
    }

    @Override 
    public void execute()
    {
        // Try to move elevator towards target position
        elevatorPosition = elevatorSubsystem.getPosition();
        var absDifference = Math.abs(elevatorPosition - target);

        if (elevatorPosition > target)
        {
            if (absDifference > 0.1)
            {
                elevatorSubsystem.move(-0.5);
            }
            else 
            {
                elevatorSubsystem.move(-0.1);
            }
        }
        else if (elevatorPosition < target)
        {
            if (absDifference > 0.1)
            {
                elevatorSubsystem.move(0.5);
            }
            else 
            {
                elevatorSubsystem.move(0.1);
            }

        }
    }

    @Override 
    public boolean isFinished()
    {
        return Math.abs(elevatorPosition - target) < 0.01;
    }
}
