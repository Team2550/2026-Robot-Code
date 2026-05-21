// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

//For kracken
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax ShooterUpper1Motor = new SparkMax(Constants.Subsystems.Shooter.kShooterUpper1Port,
      MotorType.kBrushless);
  private SparkMax ShooterUpper2Motor = new SparkMax(Constants.Subsystems.Shooter.kShooterUpper2Port,
      MotorType.kBrushless);
  private final TalonFX shooterLowerMotor = new TalonFX(Constants.Subsystems.Shooter.kShooterLowerPort);


  private final RelativeEncoder ShooterUpperEncoder = ShooterUpper2Motor.getEncoder();
  PIDController shooterPID = new PIDController(0.00003, 0.0001, 0.000017);
  private final DutyCycleOut percentOutput = new DutyCycleOut(0);
  double shooterSpeed = 2900;

  public ShooterSubsystem() {
    // Configure the PID controller with the desired gains and settings
    SmartDashboard.putNumber("Shooter Speed", 2900);
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
    shooterSpeed = SmartDashboard.getNumber("Shooter Speed", shooterSpeed);
    


  }

  public Command StartShoot() {
    return this.run(() -> {

      double shooter = shooterPID.calculate(Math.abs(ShooterUpperEncoder.getVelocity()), shooterSpeed);

      ShooterUpper1Motor.set(shooter);
      ShooterUpper2Motor.set(-shooter);
      if (Math.abs(ShooterUpperEncoder.getVelocity()) > shooterSpeed - 200) {
        shooterLowerMotor.setControl(percentOutput.withOutput(1));
      }
    });
  }

  public void StartShootVoid() {

    double shooter = shooterPID.calculate(Math.abs(ShooterUpperEncoder.getVelocity()), shooterSpeed);
    ShooterUpper1Motor.set(shooter);
    ShooterUpper2Motor.set(-shooter);
    if (Math.abs(ShooterUpperEncoder.getVelocity()) > shooterSpeed - 200) {
      shooterLowerMotor.setControl(percentOutput.withOutput(1));
    }
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
