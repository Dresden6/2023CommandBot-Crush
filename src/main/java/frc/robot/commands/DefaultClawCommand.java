package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class DefaultClawCommand extends CommandBase {
    
    final XboxController controller;
    final ClawSubsystem clawSubsystem;

    public DefaultClawCommand(XboxController controller, ClawSubsystem clawSubsystem) {
        this.controller = controller;
        this.clawSubsystem = clawSubsystem;
        this.addRequirements(clawSubsystem);
        }

    @Override
    public void execute() {

        /*
         * im not too sure how we're going to control the claw
         * we can maybe somehow set it to go up with one trigger 
         * and down with the other?
         */
        double speed = controller.getLeftTriggerAxis();
        clawSubsystem.move(speed);
    }
}
