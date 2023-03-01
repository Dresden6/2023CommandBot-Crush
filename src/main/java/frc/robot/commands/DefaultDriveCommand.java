package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    
  final XboxController controller;
  final DriveSubsystem driveSubsystem;

  public DefaultDriveCommand(XboxController controller, DriveSubsystem driveSubsystem) {
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
    this.addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

    /* 
    double turn = joystick.getRawAxis(XboxController.Axis.kRightX.value);
    double forw = joystick.getRawAxis(XboxController.Axis.kLeftY.value);

    // deadband gamepad 10% 
    if (Math.abs(forw) < 0.10) {
      forw = 0;
    }
    if (Math.abs(turn) < 0.10) {
      turn = 0;
    }

    double leftSpeed = (forw + turn);
    double rightSpeed = (forw - turn);

    */
    double fwd = controller.getLeftY();
    double rot = controller.getRightX();

    driveSubsystem.arcadeDrive(fwd, rot);

  }
}
