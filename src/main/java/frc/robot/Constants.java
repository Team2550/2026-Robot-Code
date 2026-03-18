// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


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
            public static final double kMaxBoostSpeed = 1.00;
            public static final double kMaxNormalSpeed = 0.70;
            public static final double kMaxRotSpeed = 0.5; // Only used in vision
        
            public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(5.46);
        }

        public static final class Shooter {
            // Ports
            public static final int kShooter1Port = 7;
            public static final int kShooter2Port = 0;
            // Speeds
            public static final double kMaxShooterSpeedOut1 = 0.85;// Shooter =85%
            public static final double kMaxShooterSpeedOut2 = 0.76;// Shooter = 76%
            public static final double kShooterTargetSpeed1 = 3427.0; // Target speed in RPM for the shooter
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
            public static final double kClimberDownTarget = 1.0;
        }

        public static final class Intake {
            // Ports
            public static final int kIntakePort = 12;
            // Speeds
            public static final double kMaxIntakeSpeed = 0.20; // IntakeSpeed = 32%
        }

        public static final class Vision {
            //Target
            public static final double kYawTarget = -9; //Degrees
            public static final double kDistanceTarget = 1.9; //Meters
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

            public static final Pose2d blueClimbPos = new Pose2d(
                    Units.inchesToMeters(163.86),
                    Units.inchesToMeters(21.75 + 11.46),
                    new Rotation2d(0.0));
            public static final Pose2d redClimbPos = new Pose2d(
                Units.inchesToMeters(649.57),
                Units.inchesToMeters(152.78 - 11.46),
                new Rotation2d(0.0));

            public static final Transform3d kCameraToRobot = new Transform3d(
                    -0.1016, // forward from robot center
                    0.0, // left/right camera is centered
                    0.5588, // up from floor
                    new edu.wpi.first.math.geometry.Rotation3d(0, 0.17453293, 0));
                                                                                            //3.14159265
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
