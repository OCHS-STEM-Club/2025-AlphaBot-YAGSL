// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  

  // Controller Defintions
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final XboxController m_driverController_HID = m_driverController.getHID();

  // Subsystem Definitions
  private final SwerveSubsystem m_swerveSubsystem  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  // Main Drive Definitions
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController_HID.getLeftY() * -1,
                                                                () -> m_driverController_HID.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController_HID::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);


  public RobotContainer()
  {
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      m_driverController.start().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); 

      m_driverController.b().whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());
      m_driverController.x().whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());
      m_driverController.y().whileTrue(m_swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
      m_driverController.start().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
      m_driverController.back().whileTrue(m_swerveSubsystem.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
      m_driverController.x().onTrue(Commands.runOnce(m_swerveSubsystem::addFakeVisionReading));
      m_driverController.b().whileTrue(
          m_swerveSubsystem.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      m_driverController.y().whileTrue(m_swerveSubsystem.aimAtSpeaker(2));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_swerveSubsystem.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
