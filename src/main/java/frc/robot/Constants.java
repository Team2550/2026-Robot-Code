// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static class SpeedChange {
        public static final double stickDeadband = 0.07;
        public static final double maxBoostSpeed = 1.00;
        public static final double maxNormalSpeed = 0.70;
    }

    public static final class DrivePorts {
        // Drive Motor Ports
        public static final int LEFT_MASTER = 0; // 0
        public static final int LEFT_FOLLOWER = 1; // 1
        public static final int RIGHT_MASTER = 3;
        public static final int RIGHT_FOLLOWER = 2;

    }

    public static final class SubsystemPorts {
        // CAN Ports
        // Shooter Ports
        public static final int Shooter1Port = 2; // 4
        public static final int Shooter2Port = 3; // 5
        // Climber Port
        public static final int ClimberPort = 19;
        // Intake Port
        public static final int IntakePort = 4;
        // Agitator Port
        public static final int AgitatorPort = 5;
    }

    public static final class MotorSpeeds {
        public static final double MaxShooterSpeedOut = 0.55;// Shooter = 55%
        public static final double MaxShooterSpeedIn = -0.24; // Shooter = 8%
        public static final double MaxIntakeSpeed = 0.40; // IntakeSpeed = 40%
        public static final double MaxAgitatorSpeed = 0.30; // Agitator Speed = 30%
        public static final double ClimberSpeed = 0.10; // Climber Speed = 10%
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
