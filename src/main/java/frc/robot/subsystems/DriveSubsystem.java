package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Drive subsystem that groups left/right motor controllers and exposes drive methods. */
public class DriveSubsystem extends SubsystemBase {
    private final PWMSparkMax leftMaster = new PWMSparkMax(Constants.DrivePorts.LEFT_MASTER);
    private final PWMSparkMax leftFollower = new PWMSparkMax(Constants.DrivePorts.LEFT_FOLLOWER);
    private final PWMSparkMax rightMaster = new PWMSparkMax(Constants.DrivePorts.RIGHT_MASTER);
    private final PWMSparkMax rightFollower = new PWMSparkMax(Constants.DrivePorts.RIGHT_FOLLOWER);

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftFollower);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightFollower);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    public DriveSubsystem() {
        // Configure defaults here (inversion, safety, etc.) if needed
        drive.setSafetyEnabled(true);
    }

    /** Arcade drive helper (forward, rotation). */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /** Tank drive helper (left, right). */
    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void stop() {
        drive.stopMotor();
    }

    @Override
    public void periodic() {
        // Put periodic subsystem code here (telemetry, safety checks)
    }
}
