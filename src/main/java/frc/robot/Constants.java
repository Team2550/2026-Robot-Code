// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;




public final class Constants {

    public static final class Subsystems {
        public static final class Drive {
            // Ports
            public static final int kLEFT_MASTER = 4;
            public static final int kLEFT_FOLLOWER = 6;
            public static final int kRIGHT_MASTER = 3;
            public static final int kRIGHT_FOLLOWER = 5;
            public static final int kGyroPort = 17;
            // Speeds
            public static final double kStickDeadband = 0.07;
            public static final double kMaxBoostSpeed = 0.80;
            public static final double kMaxNormalSpeed = 0.60;
            public static final double kMaxRotSpeed = 0.5; // Only used in vision
        
            public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(5.46);
        }

        public static final class Shooter {
            // Ports
            public static final int kShooterLowerPort = 26;
            public static final int kShooterUpper1Port = 7;
            public static final int kShooterUpper2Port = 12; //
            // Speeds
            public static final double kMaxShooterSpeedOut1 = 0.85;// Shooter =85%
            public static final double kMaxShooterSpeedOut2 = 0.76;// Shooter = 76%
            public static final double kShooterTargetSpeedUpper = 3427.0; // Target speed in RPM for the shooter
        }

        public static final class Climber {
            // Ports
            public static final int kClimberPort = 9;
            // Speeds
            public static final double kClimberUpSpeed = 0.20; // Climber Up Speed = 20%
            public static final double kClimberDownSpeed = -0.3; // Climber Down Speed = 30%
            public static final double kClimberManualSpeed = -0.2; // Climber Manual Speed = 20%
            // Targets
            public static final double kClimberUpTarget = 61.3;
            public static final double kClimberDownTarget = 2.0;
        }

        public static final class Intake {
            // Ports
            public static final int kIntakePort = 12; //12
            // Speeds
            public static final double kMaxIntakeSpeed = 0.20; // IntakeSpeed = 32%
        }

        public static final class Vision {
            //Target
            public static final double kYawTarget = 0.2; //Degrees
            public static final double kDistanceTarget = 3.5; //Meters
            public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout
                    .loadField(AprilTagFields.kDefaultField);

            public static final Pose2d kHubPoseBlue = new Pose2d(
                    Units.inchesToMeters(182.11),
                    Units.inchesToMeters(158.84),
                    new Rotation2d(0.0));
            public static final Pose2d kHubPoseRed = new Pose2d(
                Units.inchesToMeters(469.64),
                Units.inchesToMeters(158.84),
                new Rotation2d(0.0));


                       
            public static final Pose2d kClimbFinalRed = new Pose2d(
                Units.inchesToMeters(608.5),
                Units.inchesToMeters(146.22),
                new Rotation2d(0.0));
            public static final Pose2d kClimbFinalBlue = new Pose2d(
                Units.inchesToMeters(41.5),
                Units.inchesToMeters(164.47),
                new Rotation2d(0.0));
            public static final Pose2d kClimbFirstRed = new Pose2d(
                Units.inchesToMeters(580.03),
                Units.inchesToMeters(146.22),
                new Rotation2d(0.0));
            public static final Pose2d kClimbFirstBlue = new Pose2d(
                Units.inchesToMeters(79.33),
                Units.inchesToMeters(164.47),
                new Rotation2d(0.0));
            

            public static final Transform3d kCameraToRobot = new Transform3d(
                    -0.127, // forward from robot center     
                    0.1651, // left/right 
                    0.5588, // up from floor
                    new edu.wpi.first.math.geometry.Rotation3d(0, 0.1745329, 3.141593));

            public static final Transform3d kCameraToRobot2 = new Transform3d(
                    0.127, // forward from robot center     
                    0.1651, // left/right camera is centered
                    0.5588, // up from floor
                    new edu.wpi.first.math.geometry.Rotation3d(0, 0, 0));

            

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
                                                                                     
        }

        public static final class Agitator {
            // Ports
            public static final int kAgitatorPort = 7;//PWM
            // Speeds
            public static final double kMaxAgitatorSpeed = 0.7; // Agitator Speed = 30%
        }
    }

    // Joysticks and Buttons
    public static final class Controls {
        // Driver Joystick and Buttons
        public static final class Driver {
            public static final int kJoystickID = 0;
        }

        // Operator Joystick and Buttons
        public static final class Operator {
            public static final int kJoystickID = 1;
        }
    }

}
