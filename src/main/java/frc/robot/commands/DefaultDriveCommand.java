package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    
  final Joystick joystick;
  final DriveSubsystem driveSubsystem;

  public DefaultDriveCommand(Joystick joystick, DriveSubsystem driveSubsystem) {
    this.joystick = joystick;
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
    double left = joystick.getRawAxis(XboxController.Axis.kRightY.value);
    double right = joystick.getRawAxis(XboxController.Axis.kLeftY.value);

    driveSubsystem.drive(left, right);
  }
}
