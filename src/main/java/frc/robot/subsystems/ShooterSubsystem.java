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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

// For PWM
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Shooter1Motor = new SparkMax(Constants.Subsystems.Shooter.kShooter1Port, MotorType.kBrushless);
  private PWMVictorSPX Shooter2Motor = new PWMVictorSPX(Constants.Subsystems.Shooter.kShooter2Port);
  private final RelativeEncoder ShooterEncoder = Shooter1Motor.getEncoder();
    PIDController shooterPID = new PIDController(0.00027, 0.00017, 0.000017);


  public ShooterSubsystem() {
    // Configure the PID controller with the desired gains and settings
  
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

    SmartDashboard.putNumber("Shooter RPM", ShooterEncoder.getVelocity());

  }

  public Command StartShoot() {
    return this.run(() -> {

         double shooter = shooterPID.calculate(ShooterEncoder.getVelocity(), 5000);
      

            Shooter1Motor.set(shooter);
      if (ShooterEncoder.getVelocity() > 4000){
              Shooter2Motor.set(Constants.Subsystems.Shooter.kMaxShooterSpeedOut2);
  }});
  }

  public void StartShootVoid(){

      double shooter = shooterPID.calculate(ShooterEncoder.getVelocity(), 5000);
            Shooter1Motor.set(shooter);
      if (ShooterEncoder.getVelocity() > 2000){
              Shooter2Motor.set(Constants.Subsystems.Shooter.kMaxShooterSpeedOut2);
  }  }



  public Command StopShoot() {
    return this.run(() -> {

    

            Shooter1Motor.set(0);
    
              Shooter2Motor.set(0);
         
  });
  }



}
