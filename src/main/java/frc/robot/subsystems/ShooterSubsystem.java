// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Controls.Driver;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// For CAN
import com.revrobotics.spark.SparkMax;
import org.ejml.equation.IntegerSequence.For;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// For PWM
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Shooter1Motor;
  private SparkMax Shooter2Motor;

  public ShooterSubsystem() {
    // For CAN

    Shooter1Motor = new SparkMax(Constants.SubsystemPorts.Shooter1Port, MotorType.kBrushless);
    Shooter2Motor = new SparkMax(Constants.SubsystemPorts.Shooter2Port, MotorType.kBrushless);

    // For PWM
    // Shooter1Motor = new PWMSparkMax(Constants.SubsystemPorts.Shooter1Port);
    // Shooter2Motor = new PWMSparkMax(Constants.SubsystemPorts.Shooter2Port);

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

  public Command StartShoot() {
    return this.run(() -> {
      Shooter1Motor.set(Constants.MotorSpeeds.MaxShooterSpeedOut);
      Shooter2Motor.set(Constants.MotorSpeeds.MaxShooterSpeedOut);
    });
  }

  public Command StopShoot() {
    return this.run(() -> {
      Shooter1Motor.set(0.0);
      Shooter2Motor.set(0.0);
    });
  }

  public Command ReverseShoot() {
    return this.run(() -> {
      Shooter1Motor.set(Constants.MotorSpeeds.MaxShooterSpeedIn);
      Shooter2Motor.set(Constants.MotorSpeeds.MaxShooterSpeedIn);
    });
  }

}
