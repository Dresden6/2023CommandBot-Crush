// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SuperStructureToPos extends CommandBase {

  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  DoubleSupplier elevatorPos;
  DoubleSupplier armPos;


  /**
   * 
   * Moves the entire superstructure to the desired configuration, including the elevator and arm.
   * 
   * @param elevatorPos
   * @param armPos
   * @param elevatorSubsystem
   * @param armSubsystem
   */
  public SuperStructureToPos(DoubleSupplier elevatorPos, DoubleSupplier armPos, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.elevatorPos = elevatorPos;
    this.armPos = armPos;
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(elevatorPos.getAsDouble());
    
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
