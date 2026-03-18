// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controls.Driver;
import frc.robot.Constants.Controls.Operator;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Max speed variable for drive scaling
  double speed = Constants.Subsystems.Drive.kMaxNormalSpeed;
  public boolean m_arcade = true;

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final AgitatorSubsystem m_AgitatorSubsystem = new AgitatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final PhotonVision m_photonVision = new PhotonVision(m_driveSubsystem, m_ShooterSubsystem, m_AgitatorSubsystem, m_IntakeSubsystem, m_ClimberSubsystem);
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Driver.kJoystickID);
  private final CommandXboxController m_operatorController = new CommandXboxController(Operator.kJoystickID);
  private final SendableChooser<Command> autoChooser;

  // Drive mode: false = tank, true = arcade (Default)

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    registerNamedCommands();
    configureBindings();
    // drive command
    // if (!remoteOperated) {
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              if (m_arcade) {
                double rot = applyDeadbandAndScale(m_driverController.getRightX());
                double fwd = applyDeadbandAndScale(m_driverController.getLeftY());

                // arcadeDrive expects (fwd, rot)
                m_driveSubsystem.arcadeDrive(fwd, rot);
              } else {
                double left = applyDeadbandAndScale(m_driverController.getLeftY());
                double right = applyDeadbandAndScale(m_driverController.getRightY());
                m_driveSubsystem.tankDrive(left, right);
              }
            },
            m_driveSubsystem));
    // }

    // Toggle drive mode -- false = tank, true = arcade
    SmartDashboard.putData("Toggle Drive Mode", new InstantCommand(() -> m_arcade = !m_arcade));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Speed boost
    m_driverController.leftTrigger()
        // Speed Boost ON
        .onTrue(new InstantCommand(() -> speed = Constants.Subsystems.Drive.kMaxBoostSpeed))
        // Speed Boost OFF
        .onFalse(new InstantCommand(() -> speed = Constants.Subsystems.Drive.kMaxNormalSpeed));

  }

  private double applyDeadbandAndScale(double value) {
    if (Math.abs(value) < Constants.Subsystems.Drive.kStickDeadband) {
      return 0.0;
    }

    return Math.copySign(Math.abs(value - Constants.Subsystems.Drive.kStickDeadband) / (1.0 - Constants.Subsystems.Drive.kStickDeadband) * speed,
        value);
  }

  // Register named commands for use in PathPlanner autonomous paths
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Shooter_ON", m_ShooterSubsystem.StartShoot());
    NamedCommands.registerCommand("Shoot_1", m_ShooterSubsystem.StartShoot());
    NamedCommands.registerCommand("Climb_UP", m_ClimberSubsystem.UpClimb());
    NamedCommands.registerCommand("Climb_DOWN", m_ClimberSubsystem.DownClimb());
    NamedCommands.registerCommand("Intake_ON", m_IntakeSubsystem.StartIntake());
    NamedCommands.registerCommand("Intake_OFF", m_IntakeSubsystem.StopIntake());
    NamedCommands.registerCommand("Agitator_ON", m_AgitatorSubsystem.StartAgitator());
    NamedCommands.registerCommand("Agitator_OFF", m_AgitatorSubsystem.StopAgitator());
  };

  private void configureBindings() {
    // Driver Controls
    // Shooter control
    m_driverController.rightBumper()
        .onTrue(m_photonVision.AimShoot());

    m_operatorController.leftBumper()
        .onTrue(m_photonVision.AimClimb());
       
    // Start Shooter (constant speed)
    m_driverController.rightTrigger()
        .onTrue(Commands.parallel(m_ShooterSubsystem.StartShoot(), m_AgitatorSubsystem.StartAgitator(), m_IntakeSubsystem.StartIntake()));
      

        // m_operatorController.b()
        //  .whileTrue(m_photonVision.AimClimb())
        //  .onFalse(m_photonVision.resetClimb());

m_driverController.rightBumper().negate().and(m_driverController.rightTrigger().negate())
  .onTrue(Commands.parallel(m_ShooterSubsystem.StopShoot(), m_AgitatorSubsystem.StopAgitator(), m_IntakeSubsystem.StopIntake()));

    // Climber control
    m_operatorController.a()
        .onTrue(m_ClimberSubsystem.OverDown())

        .onFalse(m_ClimberSubsystem.StopClimb());

    // Climber Up All the way
    m_operatorController.povUp()
        .onTrue(m_ClimberSubsystem.UpClimb());


    //Gyro zero
    m_operatorController.y()
      .onTrue(m_driveSubsystem.resetPigeon());

    // Climber Down All the way
    m_operatorController.povDown()
        .onTrue(m_ClimberSubsystem.DownClimb());

    // Stop Climb
    m_operatorController.x()
        .onTrue(m_ClimberSubsystem.StopClimb());

    // Intake control
    // Start Intake
    m_operatorController.rightTrigger()
        .onTrue(m_IntakeSubsystem.StartIntake())
        .onFalse(m_IntakeSubsystem.StopIntake());





  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_photonVision.AimShoot();
  }
    public Command DriveForwardCommand() {
    // An example command will be run in autonomous
    return m_driveSubsystem.DriveForward();
  }
}
