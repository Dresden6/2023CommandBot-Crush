package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    
    final private XboxController controller;
    final private ElevatorSubsystem elevatorSubsystem;

    public DefaultElevatorCommand(XboxController controller, ElevatorSubsystem elevatorSubsystem) {
        this.controller = controller;
        this.elevatorSubsystem = elevatorSubsystem;
        this.addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        
        double speed = controller.getLeftY();
        elevatorSubsystem.move(speed);

        // TODO: Add code here to control angle of arm based on height of elevator
    }
}