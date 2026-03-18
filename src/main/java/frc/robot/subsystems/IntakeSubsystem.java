// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

// For CAN
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
//Kracken 
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX IntakeMotor = new TalonFX(Constants.Subsystems.Intake.kIntakePort);
    PIDController speedPID = new PIDController(0.0008, 0.0005, 0.00005);
    private Timer timer = new Timer();
      private final DutyCycleOut percentOutput = new DutyCycleOut(0);

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
    SmartDashboard.putNumber("Intake", IntakeMotor.getVelocity().getValueAsDouble());
  }

  public Command StartIntake() {
    return new RunCommand(() -> {
               // double intake = speedPID.calculate(IntakeMotor.getVelocity().getValueAsDouble(), 500);
      IntakeMotor.setControl(percentOutput.withOutput(0.4));
    });
  }


    public Command StartIntakeAuto() {
      timer.reset();
      timer.start();
    return new RunCommand(() -> {
                
      IntakeMotor.setControl(percentOutput.withOutput(0.4));
    }, this).until(()-> timer.hasElapsed(5))
    .finallyDo(() -> {
      IntakeMotor.setControl(percentOutput.withOutput(0));
    });
  }

  public void StartIntakeVoid(){
      IntakeMotor.setControl(percentOutput.withOutput(0.4));
      }



  public Command ReverseIntake() {
    return this.run(() -> {
      IntakeMotor.setControl(percentOutput.withOutput(-0.4));    
    });
  }

  public Command StopIntake() {
    return this.run(() -> {
      IntakeMotor.setControl(percentOutput.withOutput(0));    
    });
  }

}
