// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
// PhotonVision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;



public class PhotonVision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final AgitatorSubsystem m_AgitatorSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  private PhotonPoseEstimator photonEstimator;

  private PhotonCamera camera;

  PIDController turnPID = new PIDController(0.1, 0.005, 0.01);
  PIDController drivePID = new PIDController(1.05, 0.005, 0.1);
  PIDController climbDrivePID = new PIDController(1.05, 0.005, 0.1);
  PIDController climbTurnPID = new PIDController(0.1, 0.005, 0.01);
  Rotation2d targetYaw;
  Rotation2d climbTargetYaw;
  double distanceToTarget;
  double climbDistanceToTarget;
  double rotaioionSpeed;
  boolean climbFirst = false;

  /**
   * Construct PhotonVision with shared subsystem references.
   * 
   * @param drive   shared DriveSubsystem
   * @param shooter shared ShooterSubsystem
   * @param agitator shared AgitatorSubsystem
   */
  public PhotonVision(DriveSubsystem drive, ShooterSubsystem shooter, AgitatorSubsystem agitator, IntakeSubsystem intake, ClimberSubsystem climb) {
    this.m_driveSubsystem = drive;
    this.m_ShooterSubsystem = shooter;
    this.m_AgitatorSubsystem = agitator;
    this.m_IntakeSubsystem = intake;
    this.m_ClimberSubsystem = climb;

    camera = new PhotonCamera("MainCamera");

    turnPID.setTolerance(3); // degrees
    drivePID.setTolerance(0.1524); // meters
    climbTurnPID.setTolerance(3);
    climbDrivePID.setTolerance(Units.inchesToMeters(2));

    photonEstimator = new PhotonPoseEstimator(
      Constants.Subsystems.Vision.kAprilTagFieldLayout,
        Constants.Subsystems.Vision.kCameraToRobot);
  }

  @Override
  public void periodic() {
      RunCamera();
  }

  public void RunCamera(){
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      if (visionEst.isPresent()) {
        EstimatedRobotPose estPose = visionEst.get();
        Pose3d pose3d = estPose.estimatedPose;
        Pose2d VisionEst2d = pose3d.toPose2d();
        m_driveSubsystem.addVisionMeasurement(VisionEst2d, Timer.getFPGATimestamp());
      }
    }
  }

  public Command AimClimb() {
    return new RunCommand(() -> {
      RunCamera();
      double distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
          Constants.Subsystems.Vision.blueClimbPos);
      Rotation2d targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(),
          Constants.Subsystems.Vision.blueClimbPos);

      var allianceOptional = DriverStation.getAlliance();
      DriverStation.Alliance alliance = allianceOptional.get();

      if (alliance == DriverStation.Alliance.Red) {
        // Distance
        distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
            Constants.Subsystems.Vision.redClimbPos);
        // Rotation
        targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.redClimbPos);

      } else if (alliance == DriverStation.Alliance.Blue) {
        // Distance
        distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
            Constants.Subsystems.Vision.blueClimbPos);
    
        // Rotation
        targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.blueClimbPos);
      } else {
        System.out.println("Error loading Allance color");
      }

      System.out.println("Yaw" + targetYaw.getDegrees());
      double rotaioionSpeed = turnPID.calculate(targetYaw.getDegrees(), 0);

      double driveSpeed = drivePID.calculate(distanceToTarget, 0);

      // Clamp to safty range
      rotaioionSpeed = MathUtil.clamp(rotaioionSpeed, -0.7, 0.7);
      driveSpeed = MathUtil.clamp(driveSpeed, -1, 1);

      if (!turnPID.atSetpoint()) {
        m_driveSubsystem.arcadeDrive(0, rotaioionSpeed);
      } else {
        m_driveSubsystem.arcadeDrive(driveSpeed, 0);
      }

      System.out.println("Turn: " + turnPID.atSetpoint() + "Drive" + drivePID.atSetpoint());
      if (turnPID.atSetpoint() && drivePID.atSetpoint()) {
        m_driveSubsystem.arcadeDrive(0, 0);
        m_ClimberSubsystem.UpClimb();
        System.out.println("Climbing");
      }
    }, m_driveSubsystem, m_ClimberSubsystem);
  }

  public Command AimShoot() {
    return new RunCommand(() -> {
      RunCamera();
      double distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
          Constants.Subsystems.Vision.kHubPoseBlue);
      Rotation2d targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(),
          Constants.Subsystems.Vision.kHubPoseBlue);

      var allianceOptional = DriverStation.getAlliance();
      DriverStation.Alliance alliance = allianceOptional.get();

      if (alliance == DriverStation.Alliance.Red) {
        // Distance
        distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
            Constants.Subsystems.Vision.kHubPoseRed);
        // Rotation
        targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kHubPoseRed);

      } else if (alliance == DriverStation.Alliance.Blue) {
        // Distance
        distanceToTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
            Constants.Subsystems.Vision.kHubPoseBlue);
    
        // Rotation
        targetYaw = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kHubPoseBlue);
      } else {
        System.out.println("Error loading Allance color");
      }

      System.out.println("Yaw" + targetYaw.getDegrees());
      double rotaioionSpeed = turnPID.calculate(targetYaw.getDegrees(), Constants.Subsystems.Vision.kYawTarget);

      double driveSpeed = drivePID.calculate(distanceToTarget, Constants.Subsystems.Vision.kDistanceTarget);
  

      // Clamp to safty range
      rotaioionSpeed = MathUtil.clamp(rotaioionSpeed, -0.7,
         0.7);
      driveSpeed = MathUtil.clamp(driveSpeed, -1,
          1);

              
      if (!turnPID.atSetpoint()) {
        m_driveSubsystem.arcadeDrive(0, rotaioionSpeed);
      } else {
        m_driveSubsystem.arcadeDrive(driveSpeed, 0);
      }

      System.out.println("Turn: " + turnPID.atSetpoint() + "Drive" + drivePID.atSetpoint());
      if (turnPID.atSetpoint() && drivePID.atSetpoint()) {
        m_driveSubsystem.arcadeDrive(0, 0);
        m_ShooterSubsystem.StartShootVoid();
        m_AgitatorSubsystem.StartAgitatorVoid();
        System.out.println("Shooting");
        m_IntakeSubsystem.StartIntakeVoid();
      } 
    }, m_driveSubsystem, m_ShooterSubsystem, m_AgitatorSubsystem, m_IntakeSubsystem);
  }
}
