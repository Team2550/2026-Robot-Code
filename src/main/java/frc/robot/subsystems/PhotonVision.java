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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
// PhotonVision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;






public class PhotonVision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final AgitatorSubsystem m_AgitatorSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  private PhotonPoseEstimator photonEstimator;
  //private PhotonPoseEstimator photonEstimator2;

  private PhotonCamera camera;
 // private PhotonCamera camera2;

  PIDController turnPID = new PIDController(0.2, 0.006, 0.01);
  PIDController drivePID = new PIDController(2.2, 0.005, 0.1);
  Rotation2d targetYaw;
  Rotation2d climbTargetYaw;
  double distanceToTarget;
  double climbDistanceToTarget;
  double rotaioionSpeed;
  double time = 0;
  boolean climbFirst = false;
  private final Timer timer = new Timer();
  private Matrix<N3, N1> curStdDevs;
  Timer myTimer = new Timer();

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
   // camera2 = new PhotonCamera("SecondaryCamera");

    turnPID.setTolerance(3); // degrees
    drivePID.setTolerance(0.07); // meters

    photonEstimator = new PhotonPoseEstimator(
      Constants.Subsystems.Vision.kAprilTagFieldLayout,
        Constants.Subsystems.Vision.kCameraToRobot);
      

    // photonEstimator2 = new PhotonPoseEstimator(
    //   Constants.Subsystems.Vision.kAprilTagFieldLayout,
    //   Constants.Subsystems.Vision.kCameraToRobot2
    // );
  
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
      RunCamera();
  }



  public void RunCamera(){
    var result = camera.getLatestResult();
   // var result2 = camera2.getLatestResult();
    boolean cameraBool = false;
      
    if (result.hasTargets() ) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
     // var camera1Targets = result.getTargets().size();
     // var camera2Targets = result2.getTargets().size();


      //if (camera1Targets > camera2Targets){
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      cameraBool = false;
      // } else {
      //   visionEst = photonEstimator2.estimateCoprocMultiTagPose(result2);
      //   cameraBool = true;
      // }

      if (visionEst.isEmpty()) {
          PhotonTrackedTarget camera1Am = result.getBestTarget();
        // PhotonTrackedTarget camera2Am = result2.getBestTarget();

        // if (camera1Am.getPoseAmbiguity() < camera2Am.getPoseAmbiguity()){
        if (camera1Am.getPoseAmbiguity() < 0.3){
             visionEst = photonEstimator.estimateLowestAmbiguityPose(result);  
             cameraBool = false;
        }
        //  } else {
        //   visionEst = photonEstimator2.estimateLowestAmbiguityPose(result2);
        //   cameraBool = true;
        //  }
      
      }
    
     


//if (cameraBool){
      // updateEstimationStdDevs(visionEst, result2.getTargets(), cameraBool);
//} else {

       updateEstimationStdDevs(visionEst, result.getTargets(), cameraBool);
//}

 
       
      if (visionEst.isPresent()) {
        EstimatedRobotPose estPose = visionEst.get();
        Pose3d pose3d = estPose.estimatedPose;
        Pose2d VisionEst2d = pose3d.toPose2d();
        
        var estStdDevs = getEstimationStdDevs();
        m_driveSubsystem.addVisionMeasurement(VisionEst2d, estPose.timestampSeconds, estStdDevs);
      }
    }



  }



    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, boolean cameraNum1) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Subsystems.Vision.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Subsystems.Vision.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

          //   if (cameraNum1){
          //   // Precalculation - see how many tags we found, and calculate an average-distance metric
          //   for (var tgt : targets) {
          //       var tagPose = photonEstimator2.getFieldTags().getTagPose(tgt.getFiducialId());
          //       if (tagPose.isEmpty()) continue;
          //       numTags++;
          //       avgDist +=
          //               tagPose
          //                       .get()
          //                       .toPose2d()
          //                       .getTranslation()
          //                       .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          //   }

          // } else {
             // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }
       //   }


            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.Subsystems.Vision.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.Subsystems.Vision.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }



    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
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

          SmartDashboard.putNumber("Dis", distanceToTarget);
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


  public Command AimShootAuto() {
    return new RunCommand(() -> {
        myTimer.start();
          
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


    }, m_driveSubsystem, m_ShooterSubsystem, m_AgitatorSubsystem, m_IntakeSubsystem)
    .until(() -> myTimer.hasElapsed(5));
  }



}