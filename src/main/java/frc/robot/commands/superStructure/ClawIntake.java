// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClawIntake extends CommandBase {

  ClawSubsystem clawSubsystem;
  DoubleSupplier intakeSpeed;
  BooleanSupplier isCube;

  /**
   * Moves the elevator to the desired position
   * 
   * @param pos
   * @param isCube            If false the claw will move into the cone
   *                          configuration, if true then cube configuration
   * @param elevatorSubsystem
   */
  public ClawIntake(DoubleSupplier intakeSpeed, BooleanSupplier isCube, ClawSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.clawSubsystem = clawSubsystem;
    this.isCube = isCube;
    this.intakeSpeed = intakeSpeed;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSubsystem.runIntake(intakeSpeed.getAsDouble());

    // If the soft limits are for cube and cone, you can just move until you get to
    // the soft limit. The ClawSubsystem will stop the motor.
    if (isCube.getAsBoolean()) {
      clawSubsystem.moveClaw(0.1); // TODO Check signs
    } else {
      clawSubsystem.moveClaw(-0.1); // TODO Check signs
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.disable(); // Safety
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Don't end or the elevator will fall from where it was, and not actually hold
                  // position.
  }
}
