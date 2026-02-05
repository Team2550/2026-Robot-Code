// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
//For CAN
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// For PWM
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax ClimbMotor = new SparkMax(Constants.SubsystemPorts.ClimberPort, MotorType.kBrushless);
  private final RelativeEncoder ClimbEncoder = ClimbMotor.getEncoder();

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20); // 20 Amp current limit to protect motor and battery
    ClimbMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    ClimbEncoder.setPosition(0);

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
  }

  public Command UpClimb() {
    return this.run(() -> {

      if (ClimbEncoder.getPosition() < 50) { // Prevents the climber from going too high
        ClimbMotor.set(Constants.MotorSpeeds.ClimberSpeed);
      } else {
        ClimbMotor.set(0);
      }
    });
  }

  public Command StopClimb() {
    return this.run(() -> {
      ClimbMotor.set(0);
    });
  }

  public Command DownClimb() {
    return this.run(() -> {

      if (ClimbEncoder.getPosition() > 0) { // Prevents the climber from going too low
        ClimbMotor.set(-Constants.MotorSpeeds.ClimberSpeed);
      } else {
        ClimbMotor.set(0);
      }
    });
  }

}
