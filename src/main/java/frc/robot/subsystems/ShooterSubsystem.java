// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

//For kracken
import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax ShooterUpper1Motor = new SparkMax(Constants.Subsystems.Shooter.kShooterUpper1Port,
      MotorType.kBrushless);
  private SparkMax ShooterUpper2Motor = new SparkMax(Constants.Subsystems.Shooter.kShooterUpper2Port,
      MotorType.kBrushless);
  private final TalonFX shooterLowerMotor = new TalonFX(Constants.Subsystems.Shooter.kShooterLowerPort);

  private final RelativeEncoder ShooterUpperEncoder = ShooterUpper2Motor.getEncoder();

  // UNITS:
  // - ShooterUpperEncoder.getVelocity() returns MOTOR RPM (no velocity
  // conversion factor is set on the SparkMax), so setpoints are in RPM.
  // - Motors are driven with setVoltage(), so PID + feedforward output VOLTS.
  //
  // PID: input = RPM error, output = volts -> P is in volts per RPM.
  private double P = 0.0008;
  private double I = 0.00;
  private double D = 0.0009;

  PIDController shooterPID = new PIDController(P, I, D);

  // Feedforward: input = RPS (rotations per SECOND), output = volts.
  // kS = volts to overcome friction, kV = volts per RPS.
  // NEO free speed is ~5676 RPM = ~94.6 RPS at 12 V -> kV ~= 12 / 94.6 ~= 0.127
  // V/RPS.
  // (The old value 0.00018 was ~1/5676, i.e. duty-cycle per RPM, not volts per
  // RPS.)
  private static final double kS = 0.0;
  private static final double kV = 12.0 / (5676.0 / 60.0);
  SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(kS, kV);

  private final DutyCycleOut percentOutput = new DutyCycleOut(0);
  private final double shooterSpeed = 1000; // RPM

  public ShooterSubsystem() {
    // Configure the PID controller with the desired gains and settings
    SmartDashboard.putNumber("SHOOTER SPEED", shooterSpeed);
    SmartDashboard.putNumber("P", P);
    SmartDashboard.putNumber("I", I);
    SmartDashboard.putNumber("D", D);

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
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter RPM", Math.abs(ShooterUpperEncoder.getVelocity()));
    // P = SmartDashboard.getNumber("P", P);
    // I = SmartDashboard.getNumber("I", I);
    // D = SmartDashboard.getNumber("D", D);
    // Update gains in place (re-creating the controller every loop wipes the
    // I accumulator and D history)
    // shooterPID.setPID(P, I, D);

  }

  public Command StartShoot() {
    return this.run(() -> {

      runShooterAtRPM(3350);
    });
  }

  /** Measured shooter speed in RPM (positive). */
  private double getShooterRPM() {
    return Math.abs(ShooterUpperEncoder.getVelocity());
  }

  /**
   * Closed-loop velocity control. Feeds the lower (kicker) motor once the
   * shooter is within 150 RPM of the target.
   *
   * @param targetRPM target speed in motor RPM
   */
  private void runShooterAtRPM(double targetRPM) {
    double currentRPM = getShooterRPM();

    double ffVoltage = shooterFeedforward.calculate(targetRPM / 60.0); // RPM -> RPS
    double pidVoltage = shooterPID.calculate(currentRPM, targetRPM); // RPM in, volts out
    double volts = MathUtil.clamp(ffVoltage + pidVoltage, -12.0, 12.0);

    ShooterUpper1Motor.setVoltage(volts);
    ShooterUpper2Motor.setVoltage(-volts);

    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putNumber("Shooter Volts", volts);

    if (currentRPM > targetRPM - 150) {
      shooterLowerMotor.setControl(percentOutput.withOutput(1));
    }
  }

  public void StartShootVoid(double distance) {
    SmartDashboard.putNumber("Distance", distance);
    // if (distance != 0){
    // shooter = shooterPID.calculate(Math.abs(ShooterUpperEncoder.getVelocity()),
    // Constants.Subsystems.Shooter.kShooterSpeedMap.get(distance));
    // }
    // "SHOOTER SPEED" on the dashboard is in RPM
    runShooterAtRPM(Constants.Subsystems.Shooter.kShooterSpeedMap.get(distance));
  }

  public Command StartShootFull() {
    return this.run(() -> {
      ShooterUpper1Motor.set(1);
      ShooterUpper2Motor.set(-1);
      if (Math.abs(ShooterUpperEncoder.getVelocity()) > 5000) {
        shooterLowerMotor.setControl(percentOutput.withOutput(1));
      }
    });
  }

  public Command StopShoot() {
    return this.run(() -> {

      ShooterUpper1Motor.set(0);
      ShooterUpper2Motor.set(0);

      shooterLowerMotor.setControl(percentOutput.withOutput(0.0));

    });
  }

  public Command RevShoot() {
    return this.run(() -> {
      ShooterUpper1Motor.set(-0.5);
      ShooterUpper2Motor.set(0.5);

      shooterLowerMotor.setControl(percentOutput.withOutput(-0.73));

    });
  }

}
