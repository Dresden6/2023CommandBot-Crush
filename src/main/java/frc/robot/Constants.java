// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }

  public static final class SuperStructureSetpoints {

    // TODO Find all these values by moving them to the desired position and
    // measuring the encoder.
    public static final double elevatorHomePos = 0; // Meters
    public static final double armHomeRot = 0;

    public static final double elevatorIntakeGroundPos = 0; // Meters
    public static final double armIntakeGroundRot = 0;

    public static final double elevatorScoreConeHigh = 0; // Meters
    public static final double armScoreConeHigh = 0;

    public static final double elevatorScoreConeMid = 0; // Meters
    public static final double armScoreConeMid = 0;

  }

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double kEncoderCPR = 7.4;
    public static final double kWheelDiameterMeters = 0.127;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian,
        kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class ElevatorConstants {

    public static final int leftElevatorMotorPort = 7;
    public static final int rightElevatorMotorPort = 8;

    // TODO Tune all of these values
    public static final double elevatorPIDTolerance = 0.02; // Meters

    public static final double elevatorMaxVel = 2; // Meters per second
    public static final double elevatorMaxAccel = 1; // Meters per second squared

    // TODO Tune KP
    // To tune P, you should start with a very low value like 0.1, and then slowly
    // increase it until the elevator starts to oscillate around the setpoint. From
    // here, you can try decreasing it a tiny amount and stick with that.
    public static final double elevatorKP = 0;
    public static final double elevatorKI = 0; // Keep KI at 0
    public static final double elevatorKD = 0; // Probably keep KD at 0 too

    // This is good for learning how to tune PID and FF:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    // It is the most similar to an elevator, but there are some differences.

    // TODO Tune Feed Forward Value
    // To do this, go to robotContainer and use Shuffleboard to find the minimum
    // voltage needed to counteract gravity. AKA, if you just gave the motor KF
    // volts, it would stay in place perfectly.
    public static final double elevatorKF = 0; // Volts.

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class ArmConstants {
    public static final int armMotorPort = 9;

    // TODO Tune tolerance
    public static final double armPIDTolerance = 0.05; // Rotations

    // TODO Tune P
    // Follow steps for Elevator
    public static final double armKP = 0;
    public static final double armKI = 0; // Keep KI at 0
    public static final double armKD = 0; // Probably keep KD at 0 too

    // TODO Tune KF
    // Find the minimum voltage needed to counteract gravity when the arm is
    // straight out horizontally, so when the motor has to work the hardest.
    public static final double armKF = 0; // Volts.

  }

  public static final class ClawConstants {
    public static final int gripMotorPort = 11;
    public static final int intakeMotorPort = 10;

    public static final double intakeSpeed = 0.5;
    public static final double outtakeSpeed = -0.5;

  }

}
