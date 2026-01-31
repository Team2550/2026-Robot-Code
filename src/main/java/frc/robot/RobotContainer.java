// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.Controls.Driver;
import frc.robot.Constants.SpeedChange;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Max speed variable for drive scaling
   double speed = 0.6;
    private boolean m_arcade = true;



  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Driver.kJoystickID);

  // Drive mode: false = tank (default), true = arcade
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings

  
    configureBindings();
    //  drive command
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              if (m_arcade) {
                double rot = applyDeadbandAndScale(m_driverController.getRightX());
                double fwd = applyDeadbandAndScale(m_driverController.getLeftY());
                m_driveSubsystem.arcadeDrive(rot, fwd);
              } else {
                double left = applyDeadbandAndScale(-m_driverController.getLeftY());
                double right = applyDeadbandAndScale(m_driverController.getRightY());
                m_driveSubsystem.tankDrive(left, right);
              }
            },
            m_driveSubsystem));

    // Toggle drive mode with the A button (press to toggle)
      m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_arcade = !m_arcade));



    m_driverController.leftTrigger()
    .onTrue(new InstantCommand(() -> speed = 1.0))
    .onFalse(new InstantCommand(() -> speed = 0.6));
  }

  private double applyDeadbandAndScale(double value) {
    if (Math.abs(value) < SpeedChange.stickDeadband) {
      return 0.0;
    }



    return Math.copySign(Math.abs(value - SpeedChange.stickDeadband) / (1.0 - SpeedChange.stickDeadband) * speed, value);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // new Trigger(m_ShooterSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_ShooterSubsystem));



    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.rightTrigger()
    .onTrue(m_ShooterSubsystem.StartMotor())
   .onFalse(m_ShooterSubsystem.StopMotor());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_ShooterSubsystem);
  }
}
