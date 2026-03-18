// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX ClimbMotor = new TalonFX(Constants.Subsystems.Climber.kClimberPort);
  // Duty cycle control request (percent output)
  // private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  double positionRotations = ClimbMotor.getPosition().getValueAsDouble();
  private final DutyCycleOut percentOutput = new DutyCycleOut(0);
  // private final VelocityDutyCycle velocityControl = new VelocityDutyCycle(0);

  boolean STOP = false;

  public ClimberSubsystem() {

    SmartDashboard.putData("Zero Climber", new InstantCommand(() -> ClimbMotor.setPosition(0.0)));

    // Make the motor brake when no power is applied
    // ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
    ClimbMotor.setPosition(0.0);
    ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
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

    SmartDashboard.putNumber("Climber Position (Rotations)", ClimbMotor.getPosition().getValueAsDouble());
  }

  public Command StopClimb() {
    return this.run(() -> {
      ClimbMotor.setControl(percentOutput.withOutput(0.0));
    });
  }

  public Command UpClimb() {
    return this.run(() -> {

      ClimbMotor.setControl(percentOutput.withOutput(Constants.Subsystems.Climber.kClimberUpSpeed));
    })
        .until(() -> ClimbMotor.getPosition().getValueAsDouble() >= Constants.Subsystems.Climber.kClimberUpTarget)
        .finallyDo(() -> {
          ClimbMotor.setControl(percentOutput.withOutput(0.0));

        });
  }

  public Command OverDown() {
    return this.run(() -> {
      ClimbMotor.setControl(percentOutput.withOutput(Constants.Subsystems.Climber.kClimberManualSpeed));

    });
  }

  public Command DownClimb() {
    return this.run(() -> {

      ClimbMotor.setControl(percentOutput.withOutput(Constants.Subsystems.Climber.kClimberDownSpeed));
    })
        .until(() -> ClimbMotor.getPosition().getValueAsDouble() <= Constants.Subsystems.Climber.kClimberDownTarget)
        .finallyDo(() -> {
          ClimbMotor.setControl(percentOutput.withOutput(0.0));

        });

  }
}
