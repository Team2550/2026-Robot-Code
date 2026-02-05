// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// For CAN
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// For PWM
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class AgitatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax AgitatorMotor;

  public AgitatorSubsystem() {
    AgitatorMotor = new SparkMax(Constants.SubsystemPorts.AgitatorPort, MotorType.kBrushless);

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

  public Command StartAgitator() {
    return this.run(() -> {
      AgitatorMotor.set(Constants.MotorSpeeds.MaxAgitatorSpeed);
    });
  }

  public Command ReverseAgitator() {
    return this.run(() -> {
      AgitatorMotor.set(-Constants.MotorSpeeds.MaxAgitatorSpeed);
    });
  }

  public Command StopAgitator() {
    return this.run(() -> {
      AgitatorMotor.set(0);
    });
  }

}
