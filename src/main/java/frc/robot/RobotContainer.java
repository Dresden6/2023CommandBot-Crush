// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SuperStructureSetpoints;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.superStructure.ArmToPos;
import frc.robot.commands.superStructure.ClawIntake;
import frc.robot.commands.superStructure.ElevatorToPos;
import frc.robot.commands.superStructure.SuperStructureToPos;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();


  private GenericEntry elevatorOutputVolts;
  private GenericEntry armOutputVolts;

  private boolean isIntakeConfigCube = false; // True for cone, false for cube

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController controller =
      new XboxController(OperatorConstants.kDriverControllerPort);

  private final XboxController armController = 
    new XboxController(OperatorConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.init();
    driveSubsystem.register();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(controller, driveSubsystem));

    clawSubsystem.init();
    clawSubsystem.register();
    // clawSubsystem.setDefaultCommand(new DefaultClawCommand(armController, clawSubsystem));

    elevatorSubsystem.init();
    elevatorSubsystem.register();
    elevatorSubsystem.setDefaultCommand(new ElevatorToPos(() -> SuperStructureSetpoints.elevatorHomePos, elevatorSubsystem));

    armSubsystem.init();
    armSubsystem.register();
    armSubsystem.setDefaultCommand(new ArmToPos(() -> SuperStructureSetpoints.armHomeRot, armSubsystem));
    
    ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");


    // Comment out both of these poseFinder things when you're done tuning
    // TODO Use this to tune KG for the elevator
    elevatorOutputVolts = poseFinder.add("Elevator Volts", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("`min", 0, "max", 12, "Block increment", 0.05)).getEntry();


    // TODO Use this to tune KG for the arm
    armOutputVolts = poseFinder.add("Arm Volts", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("`min", 0, "max", 12, "Block increment", 0.05)).getEntry();


    // Configure the trigger bindings
    configureBindings();
  }


  private Trigger armControllerA = new JoystickButton(armController, XboxController.Button.kA.value);
  private Trigger armControllerB = new JoystickButton(armController, XboxController.Button.kB.value);
  private Trigger armControllerX = new JoystickButton(armController, XboxController.Button.kX.value);
  private Trigger armControllerY = new JoystickButton(armController, XboxController.Button.kY.value);
  
  private Trigger armControllerLeftBumper = new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
  private Trigger armControllerRightBumper = new JoystickButton(armController, XboxController.Button.kRightBumper.value);

  private Trigger armControllerLeftTrigger = new Trigger(() -> armController.getLeftTriggerAxis() > 0.1);
  private Trigger armControllerRightTrigger = new Trigger(() -> armController.getRightTriggerAxis() > 0.1);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Bindings for the arm controller

  
    // TODO Use this code to tune elevator and arm. You need to comment out the other code that uses the A and B buttons.
    // armControllerA.whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorVolts(elevatorOutputVolts.getDouble(0)))).onFalse(new InstantCommand(() -> elevatorSubsystem.disable()));
    // armControllerB.whileTrue(new RunCommand(() -> armSubsystem.setArmVolts(armOutputVolts.getDouble(0)))).onFalse(new InstantCommand(() -> armSubsystem.disable()));


    armControllerLeftTrigger.whileTrue(new StartEndCommand(() -> isIntakeConfigCube = true, () -> isIntakeConfigCube = false));
    

    armControllerX.whileTrue(new SuperStructureToPos(() -> SuperStructureSetpoints.elevatorIntakeGroundPos, () -> SuperStructureSetpoints.armIntakeGroundRot, elevatorSubsystem, armSubsystem));
    armControllerY.whileTrue(new SuperStructureToPos(
      () -> isIntakeConfigCube ? SuperStructureSetpoints.elevatorScoreConeHigh : SuperStructureSetpoints.elevatorScoreConeHigh, 
      () -> SuperStructureSetpoints.armScoreConeHigh,
       elevatorSubsystem, armSubsystem));
    armControllerB.whileTrue(new SuperStructureToPos(() -> SuperStructureSetpoints.elevatorScoreConeMid, () -> SuperStructureSetpoints.armScoreConeMid, elevatorSubsystem, armSubsystem));

    
    armControllerLeftBumper.whileTrue(new ClawIntake(() -> ClawConstants.intakeSpeed, () -> true, clawSubsystem));
    armControllerLeftBumper.whileTrue(new ClawIntake(() -> ClawConstants.outtakeSpeed, () -> true, clawSubsystem));


    // Bindings for the driver controller 
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ParallelCommandGroup(
      new SuperStructureToPos(() -> SuperStructureSetpoints.elevatorScoreConeHigh, () -> SuperStructureSetpoints.armScoreConeHigh, elevatorSubsystem, armSubsystem), // Move superstructure
      new ClawIntake(() -> elevatorSubsystem.isAtGoal() && true ? ClawConstants.outtakeSpeed : 0, () -> true, clawSubsystem) // Outtake the cone when the setpoints are reached
    );
  }
}
