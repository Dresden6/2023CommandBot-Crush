package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    double WHEEL_DIAMETER = Units.inchesToMeters(4); // 4 inches
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    double WHEEL_GEAR_RATIO = .75;

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

    CANSparkMax motorFrontLeft;
    CANSparkMax motorFrontRight;
    CANSparkMax motorRearLeft;
    CANSparkMax motorRearRight;
  
    Gyro gyro;

    DifferentialDrive diffDrive;

    Field2d field = new Field2d();

    Encoder leftEncoder;
    Encoder rightEncoder;

    // Simulation specific items 
    DifferentialDrivetrainSim diffDriveSim;
    EncoderSim leftEncoderSim;
    EncoderSim rightEncoderSim;


    public void init() {
        motorFrontLeft = new CANSparkMax(1, MotorType.kBrushless);
        motorFrontRight = new CANSparkMax(2, MotorType.kBrushless);
        motorRearLeft = new CANSparkMax(3, MotorType.kBrushless);
        motorRearRight = new CANSparkMax(4, MotorType.kBrushless);

        motorFrontLeft.getEncoder().setPositionConversionFactor(WHEEL_GEAR_RATIO);
        motorFrontRight.getEncoder().setPositionConversionFactor(WHEEL_GEAR_RATIO);
        motorRearLeft.getEncoder().setPositionConversionFactor(WHEEL_GEAR_RATIO);
        motorRearRight.getEncoder().setPositionConversionFactor(WHEEL_GEAR_RATIO);

        motorRearLeft.follow(motorFrontLeft);
        motorRearRight.follow(motorFrontRight);

        motorFrontRight.setInverted(true);
        motorRearRight.setInverted(true);

        gyro = new AHRS(SPI.Port.kMXP, (byte)200);

        leftEncoder = new Encoder(
            DriveConstants.kLeftEncoderPorts[0],
            DriveConstants.kLeftEncoderPorts[1],
            DriveConstants.kLeftEncoderReversed);
        
        rightEncoder = new Encoder(
            DriveConstants.kRightEncoderPorts[0],
            DriveConstants.kRightEncoderPorts[1],
            DriveConstants.kRightEncoderReversed);

        leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        diffDrive = new DifferentialDrive(motorFrontLeft, motorFrontRight);

        if (Robot.isSimulation()) {
            diffDriveSim = new DifferentialDrivetrainSim(
                DriveConstants.kDrivetrainPlant,
                DriveConstants.kDriveGearbox,
                DriveConstants.kDriveGearing,
                DriveConstants.kTrackwidthMeters,
                DriveConstants.kWheelDiameterMeters / 2.0,
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
                
            /* 
            REVPhysicsSim.getInstance().addSparkMax(motorFrontLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorFrontRight, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorRearLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorRearRight, DCMotor.getNEO(1));
            */
            
            SmartDashboard.putData("Field", field);

        }

    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());
        super.periodic();
    }

    

    @Override
    public void simulationPeriodic() {
        var left = motorFrontLeft.get();
        var right = motorFrontRight.get();

        SmartDashboard.putNumber("left", motorFrontLeft.get() * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("right", motorFrontRight.get() * RobotController.getBatteryVoltage());

        diffDriveSim.setInputs(
            motorFrontLeft.get() * RobotController.getBatteryVoltage(), 
            motorFrontRight.get() * RobotController.getBatteryVoltage());
        
        
        diffDriveSim.update(0.020);
        //REVPhysicsSim.getInstance().run();

        leftEncoderSim.setDistance(diffDriveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(diffDriveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(diffDriveSim.getRightPositionMeters());
        rightEncoderSim.setRate(diffDriveSim.getRightVelocityMetersPerSecond());
        
    }

    public void drive(double leftOutput, double rightOutput) {
        diffDrive.tankDrive(leftOutput, rightOutput);
    }
    
}
