// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// For CAN
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
// For PWM
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax IntakeMotor = new SparkMax(Constants.Subsystems.Intake.kIntakePort, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = IntakeMotor.getEncoder();
    PIDController speedPID = new PIDController(0.0008, 0.0005, 0.00005);

  public IntakeSubsystem() {
    

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
    SmartDashboard.putNumber("Intake", intakeEncoder.getVelocity());
  }

  public Command StartIntake() {
    return this.run(() -> {
                double intake = speedPID.calculate(intakeEncoder.getVelocity(), 900);
      IntakeMotor.set(intake);
    });
  }


  public void StartIntakeVoid(){
      IntakeMotor.set(Constants.Subsystems.Intake.kMaxIntakeSpeed);
  }



  public Command ReverseIntake() {
    return this.run(() -> {
      IntakeMotor.set(-Constants.Subsystems.Intake.kMaxIntakeSpeed);
    });
  }

  public Command StopIntake() {
    return this.run(() -> {
      IntakeMotor.set(0);
    });
  }

}
