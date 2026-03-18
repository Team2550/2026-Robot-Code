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
import edu.wpi.first.math.kinematics.*;


import com.ctre.phoenix6.hardware.Pigeon2; //Gyro

public class DriveSubsystem extends SubsystemBase {

private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.546);

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
    double leftPosition = 0;
    double rightPosition = 0;
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
            (speeds) -> driveRobotRelative(speeds),
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
        return get2dPose();
    }

    public void resetPose(Pose2d pose) {
        // Convert encoder positions (rotations) to meters before resetting the
        // pose estimator. Encoders return rotations for REV relative encoders
        // by default; divide by the gear ratio and multiply by wheel
        // circumference to get meters.
        double leftMeters = leftEncoder.getPosition() / 8.45 * wheelCircumference;
        double rightMeters = rightEncoder.getPosition() / 8.45 * wheelCircumference;

        m_poseEstimator.resetPosition(
            pigeon.getRotation2d(),
            leftMeters,
            rightMeters,
            pose
        );
    }

    // PathPlanner expects a supplier of chassis speeds (robot-relative velocities) for
    // its AutoBuilder. Return ChassisSpeeds here.
    public ChassisSpeeds getRobotRelativeSpeeds() {

        // Encoders return RPM for REV relative encoders by default; convert to
        // meters per second: (rotations per minute) / 60 = rotations per second;
        // multiply by wheel circumference, divide by gear ratio (8.45)
        double leftMPS = leftEncoder.getVelocity() / 8.45 * wheelCircumference / 60.0;
        double rightMPS = rightEncoder.getVelocity() / 8.45 * wheelCircumference / 60.0;

        var wheelSpeeds = new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);

        return kinematics.toChassisSpeeds(wheelSpeeds);
    }


    // Accept chassis speeds from PathPlanner; convert to wheel speeds and apply
    // simple open-loop outputs (feedforward from the library is currently ignored).
    public void driveRobotRelative(ChassisSpeeds speeds) {
       DifferentialDriveWheelSpeeds wheelSpeedsDrive = kinematics.toWheelSpeeds(speeds);

    double leftPer = wheelSpeedsDrive.leftMetersPerSecond / 7.0;
    double rightPer = wheelSpeedsDrive.rightMetersPerSecond / 7.0;

     // Clamp to [-1, 1] to avoid sending invalid outputs to motors.
     leftPer = Math.max(-1.0, Math.min(1.0, leftPer));
     rightPer = Math.max(-1.0, Math.min(1.0, rightPer));

     SmartDashboard.putNumber("leftPer", leftPer);
     SmartDashboard.putNumber("rightPer", rightPer);
     // Drive the motors during path following. We negate because motor
     // inversion or drivetrain configuration may require it.
     drive.tankDrive(-leftPer, -rightPer);
     // Feed the watchdog for the DifferentialDrive to prevent safety timeouts.
     drive.feed();
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

         leftPosition = leftPos - leftEncoder.getPosition() / 8.45 * wheelCircumference;
         rightPosition = rightPos - rightEncoder.getPosition() / 8.45 * wheelCircumference;
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
