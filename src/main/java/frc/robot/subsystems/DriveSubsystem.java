package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.PhotonVision;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2; //Gyro

public class DriveSubsystem extends SubsystemBase {



    private final Field2d field = new Field2d();

    private final Pigeon2 pigeon = new Pigeon2(Constants.Subsystems.Drive.kGyroPort);

    private SparkMax leftMaster = new SparkMax(
            Constants.Subsystems.Drive.kLEFT_MASTER, MotorType.kBrushless);
    private SparkMax leftFollower = new SparkMax(
            Constants.Subsystems.Drive.kLEFT_FOLLOWER, MotorType.kBrushless);
    private SparkMax rightMaster = new SparkMax(
            Constants.Subsystems.Drive.kRIGHT_MASTER, MotorType.kBrushless);
    private SparkMax rightFollower = new SparkMax(
            Constants.Subsystems.Drive.kRIGHT_FOLLOWER, MotorType.kBrushless);
    // Encoders
    private final RelativeEncoder rightEncoder = rightMaster.getEncoder();
    private final RelativeEncoder leftEncoder = leftMaster.getEncoder();

    boolean back = false;
    boolean done = false;
    double leftDis = 0;
    double leftPos = 0;
    double rightPos = 0;
    double rightDis = 0;
    double wheelCircumference = Math.PI * 0.2032; // 8 inch diameter in meters

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftFollower);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightFollower);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    private Pose2d currentPose = new Pose2d();

    private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
            Constants.Subsystems.Drive.kinematics,
            pigeon.getRotation2d(),
            leftEncoder.getPosition() / 8.45 * wheelCircumference,
            rightEncoder.getPosition() / 8.45 * wheelCircumference,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    public DriveSubsystem() {


        SmartDashboard.putData("Field", field);
        leftMaster.setInverted(true);
        leftFollower.setInverted(true);

        pigeon.setYaw(0.0);

        drive.setSafetyEnabled(true);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> driveRobotRelative(speeds),
                    new PPLTVController(0.02),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
            // RobotConfig config = new RobotConfig(55, 4.6, null, 0.66);

        }
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }



    public Command DriveForward() {
        leftDis = 0;
        rightDis = 0;
        done = false;

        return new RunCommand(() -> {
            if (leftDis < 0.3 && !back) {
                drive.arcadeDrive(-1, 0);
                SmartDashboard.putNumber("Dis", leftDis);
            } else if (leftDis > -0.3) {
                  back = true;
                drive.arcadeDrive(1, 0);

            } else {
              
                drive.arcadeDrive(0, 0);
                done = true;
               
               
            }
                
        }, this).until(() -> done);
       
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void stop() {
        drive.stopMotor();
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public void resetPose(Pose2d pose) {
        currentPose = pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {

        Double yaw = pigeon.getYaw().getValueAsDouble();
        double yawRateRadPerSec = Math.toRadians(yaw);
        double averageSpeed = ((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2.0)
                * (wheelCircumference / 8.45 / 60.0);
        return new ChassisSpeeds(averageSpeed, 0, yawRateRadPerSec);
        // Vy is 0 because we are only doing differential drive, so we can't move
        // sideways. Vx is the speed of our drive train, omega is the rate of rotation
        // from the gyro.
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive.arcadeDrive(
                speeds.vxMetersPerSecond,
                speeds.omegaRadiansPerSecond);
    }

    public Command resetPigeon() {
        return this.run(() -> {

            pigeon.setYaw(0.0);
            System.out.println("Pigeon initialized and yaw reset.");
        });

    }

    // public Command driveForward(){
    // if ()
    // // drive.arcadeDrive(-0.4, 0);

    // }

    public Pose2d get2dPose() {
        Pose2d pose = m_poseEstimator.getEstimatedPosition();
        return pose;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    @Override
    public void periodic() {
        // Put periodic subsystem code here (telemetry, safety checks)
        m_poseEstimator.update(
                pigeon.getRotation2d(), leftDis, rightDis);

        double leftPosition = leftPos - leftEncoder.getPosition() / 8.45 * wheelCircumference;
        double rightPosition = rightPos - rightEncoder.getPosition() / 8.45 * wheelCircumference;
        leftPos = leftEncoder.getPosition() / 8.45 * wheelCircumference;
        rightPos = rightEncoder.getPosition() / 8.45 * wheelCircumference;

        leftDis = leftDis + leftPosition;
        rightDis = rightDis + rightPosition;
        SmartDashboard.putNumber("Dis", rightDis);

        boolean isHot = leftMaster.getMotorTemperature() > 50 || rightMaster.getMotorTemperature() > 50
                || leftFollower.getMotorTemperature() > 50 || rightFollower.getMotorTemperature() > 50;
        SmartDashboard.putBoolean("Drive Overheating", isHot);

        SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw().getValueAsDouble());

        Pose2d pose = m_poseEstimator.getEstimatedPosition();

        field.setRobotPose(pose);
    }
}
