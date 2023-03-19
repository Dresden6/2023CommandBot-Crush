// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmToPos extends CommandBase {

  ArmSubsystem armSubsystem;
  DoubleSupplier pos;

  /** Creates a new ElevatorToPos. */
  public ArmToPos(DoubleSupplier pos, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.pos = pos;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmPosition(pos.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.disable(); // Safety
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Don't end or the elevator will fall from where it was, and not actually hold position.
  }
}
