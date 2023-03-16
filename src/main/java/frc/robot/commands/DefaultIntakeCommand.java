package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {
    
    final private XboxController controller;
    final private IntakeSubsystem subsystem;

    public DefaultIntakeCommand(XboxController controller, IntakeSubsystem subsystem)
    {
        this.controller = controller;
        this.subsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        var isPressed = controller.getRightBumper();

        if (isPressed)
        {
            subsystem.activate();
        }
        else 
        {
            subsystem.deactivate();
        }
    }
}
