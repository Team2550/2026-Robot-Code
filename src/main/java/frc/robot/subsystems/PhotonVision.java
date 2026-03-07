// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  PIDController turnPID = new PIDController(0.08, 0.0, 0);
  PIDController drivePID = new PIDController(0.6, 0, 0);
  PIDController climbTurnPID = new PIDController(0.08, 0, 0);
  PIDController climbDrivePID = new PIDController(0.6, 0, 0);
  Rotation2d targetYaw;
  double distanceToTarget;
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
    drivePID.setTolerance(0.06); // meters
    climbDrivePID.setTolerance(0.01778); 
    climbTurnPID.setTolerance(0.03490659);

    photonEstimator = new PhotonPoseEstimator(
      Constants.Subsystems.Vision.kAprilTagFieldLayout,
        Constants.Subsystems.Vision.kCameraToRobot);
  }

  
    

  /**
   * Example command factory method.
   *
   * @return a command
   * 
   *         public Command exampleMethodCommand() {
   *         // Inline construction of command goes here.
   *         // Subsystem::RunOnce implicitly requires `this` subsystem.
   *         return runOnce(
   *         () -> {
   *         one-time action goes here
   *         });
   *         }
   * 
   *         /**
   *         An example method querying a boolean state of the subsystem (for
   *         example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   *         public boolean exampleCondition() {
   *         // Query some boolean state, such as a digital sensor.
   *         return false;
   *         }
   */
  @Override
  public void periodic() {
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

  public Command ClimbFalse(){
    return new RunCommand(() -> {
        climbFirst = false;
    } );
  }




  public Command AimShoot() {
    return new RunCommand(() -> {
  
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
            m_ShooterSubsystem.StartShoot();
            m_AgitatorSubsystem.StartAgitator();
            m_IntakeSubsystem.StartIntake();
          } else {
            m_ShooterSubsystem.StopShoot();
            m_AgitatorSubsystem.StopAgitator();
            m_IntakeSubsystem.StopIntake();
          }


    }, m_driveSubsystem, m_ShooterSubsystem, m_AgitatorSubsystem, m_IntakeSubsystem);
  }






  public Command AimClimb() {
    return new RunCommand(() -> {
          
      if (!climbFirst){
      
      //Put climber up
          m_ClimberSubsystem.UpClimb();
          
  
          double distanceToFirstTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
              Constants.Subsystems.Vision.kClimbFirstBlue);
          Rotation2d targetYawFirst = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(),
              Constants.Subsystems.Vision.kClimbFirstBlue);





          var allianceOptional = DriverStation.getAlliance();
          DriverStation.Alliance alliance = allianceOptional.get();

          if (alliance == DriverStation.Alliance.Red) {
            // Distance
            distanceToFirstTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
                Constants.Subsystems.Vision.kClimbFirstRed);
            // Rotation
            targetYawFirst = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kClimbFirstRed);

          } else if (alliance == DriverStation.Alliance.Blue) {
            // Distance
            distanceToFirstTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
                Constants.Subsystems.Vision.kClimbFirstBlue);
    
            // Rotation
            targetYawFirst = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kClimbFirstBlue);
          } else {
            System.out.println("Error loading Allance color");
          }

          System.out.println("Yaw" + targetYaw.getDegrees());
          double rotaioionSpeed = climbTurnPID.calculate(targetYawFirst.getDegrees(), 0);

          double driveSpeed = climbDrivePID.calculate(distanceToFirstTarget, 0);
  
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

          System.out.println("Turn: " + climbTurnPID.atSetpoint() + "Drive" + climbDrivePID.atSetpoint());
          if (climbTurnPID.atSetpoint() && climbDrivePID.atSetpoint()) {
            climbFirst = true;
          } 


        } else {


                 double distanceToFinalTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
              Constants.Subsystems.Vision.kClimbFirstBlue);
          Rotation2d targetYawFinal = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(),
              Constants.Subsystems.Vision.kClimbFirstBlue);



          var allianceOptional = DriverStation.getAlliance();
          DriverStation.Alliance alliance = allianceOptional.get();

          if (alliance == DriverStation.Alliance.Red) {
            // Distance
            distanceToFinalTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
                Constants.Subsystems.Vision.kClimbFinalRed);
            // Rotation
            targetYawFinal = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kClimbFinalRed);

          } else if (alliance == DriverStation.Alliance.Blue) {
            // Distance
            distanceToFinalTarget = PhotonUtils.getDistanceToPose(m_driveSubsystem.get2dPose(),
                Constants.Subsystems.Vision.kClimbFinalBlue);
    
            // Rotation
            targetYawFinal = PhotonUtils.getYawToPose(m_driveSubsystem.get2dPose(), Constants.Subsystems.Vision.kClimbFinalBlue);
          } else {
            System.out.println("Error loading Allance color");
          }

        
          double rotaioionSpeed = climbTurnPID.calculate(targetYawFinal.getDegrees(), 0);

          double driveSpeed = climbDrivePID.calculate(distanceToFinalTarget, 0);
  
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

          System.out.println("Turn: " + climbTurnPID.atSetpoint() + "Drive" + climbDrivePID.atSetpoint());
          if (climbTurnPID.atSetpoint() && climbDrivePID.atSetpoint()) {
            m_ClimberSubsystem.DownClimb();
          } 
        }
    }, m_driveSubsystem, m_ClimberSubsystem);
  
  }





}