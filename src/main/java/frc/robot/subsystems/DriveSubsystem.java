package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    double WHEEL_DIAMETER = Units.inchesToMeters(5); // 4 inches
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    double WHEEL_GEAR_RATIO = .75;

    DifferentialDriveOdometry odometry;
  
    Gyro gyro;

    MotorControllerGroup motorControllerGroupLeft;
    MotorControllerGroup motorControllerGroupRight;
    DifferentialDrive diffDrive;

    Field2d field = new Field2d();

    Encoder leftEncoder;
    Encoder rightEncoder;

    CANSparkMax leftLeader;
    CANSparkMax rightLeader;

    // Simulation specific items 
    DifferentialDrivetrainSim diffDriveSim;
    EncoderSim leftEncoderSim;
    EncoderSim rightEncoderSim;


    public void init() {
        // TODO: renumber these motor controllers with the correct ID's
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        motorControllerGroupLeft = new MotorControllerGroup(
        leftLeader,
            new CANSparkMax(3, MotorType.kBrushless),
            new CANSparkMax(5, MotorType.kBrushless));
        
        rightLeader = new CANSparkMax(2, MotorType.kBrushless);
        motorControllerGroupRight = new MotorControllerGroup(
            rightLeader,
            new CANSparkMax(4, MotorType.kBrushless),
            new CANSparkMax(6, MotorType.kBrushless));
            leftLeader.getEncoder().setPosition(0);
            rightLeader.getEncoder().setPosition(0);

        motorControllerGroupRight.setInverted(true);

        diffDrive = new DifferentialDrive(motorControllerGroupRight, motorControllerGroupLeft);

        gyro = new AHRS(SPI.Port.kMXP);

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

        resetEncoders();

        //odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        odometry.update(gyro.getRotation2d(), leftLeader.getEncoder().getPosition(), rightLeader.getEncoder().getPosition());


        
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
        //odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        odometry.update(gyro.getRotation2d(), leftLeader.getEncoder().getPosition(), rightLeader.getEncoder().getPosition());
        field.setRobotPose(odometry.getPoseMeters());

        // TANK DRIVE CODE 
        /*
         * motorFrontLeft.set(ControlMode.PercentOutput, leftY);
         * motorFrontRight.set(ControlMode.PercentOutput, rightY);
         */
    }

    @Override
    public void simulationPeriodic() {
        var left = motorControllerGroupLeft.get();
        var right = motorControllerGroupRight.get();

        SmartDashboard.putNumber("left", left * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("right", right * RobotController.getBatteryVoltage());

        diffDriveSim.setInputs(
            left * RobotController.getBatteryVoltage(), 
            right * RobotController.getBatteryVoltage());
        
        
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

    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }
    
    public void arcadeDrive(double fwd, double rot)
    {
        diffDrive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double left, double right)
    {
        motorControllerGroupLeft.setVoltage(left);
        motorControllerGroupRight.setVoltage(right);
        diffDrive.feed();
    }

    public double getAverageEncoderDistance()
    {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    public void setMaxOutput(double maxOutput)
    {
        diffDrive.setMaxOutput(maxOutput);
    }

    public void zeroHeading()
    {
        gyro.reset();
    }

    public double getHeading()
    {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate()
    {
        return -gyro.getRate();
    }
}
