// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToPos extends CommandBase {

  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier pos;

  /**
   * Moves the elevator to the desired position
   * 
   * @param pos
   * @param elevatorSubsystem
   */
  public ElevatorToPos(DoubleSupplier pos, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.elevatorSubsystem = elevatorSubsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(pos.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.disable(); // Safety
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Don't end or the elevator will fall from where it was, and not actually hold position.
  }
}
