// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Choosers
  private final SendableChooser<Command> autoChooser;

  // Controller Definitions
  final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Trigger DRIVER_A_BUTTON = new Trigger(() -> m_driverController.getHID().getAButton());
  private final Trigger DRIVER_B_BUTTON = new Trigger(() -> m_driverController.getHID().getBButton());
  private final Trigger DRIVER_X_BUTTON = new Trigger(() -> m_driverController.getHID().getXButton());
  private final Trigger DRIVER_Y_BUTTON = new Trigger(() -> m_driverController.getHID().getYButton());

  private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  private final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.leftTrigger());
  private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.rightTrigger());
  private final Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.leftBumper());
  private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.rightBumper());


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocityRedAlliance = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRawAxis(4) * -1)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRawAxis(2) * -1)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

   SwerveInputStream driveAngularVelocityBlueAlliance = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRawAxis(4) * -1)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocityRedAlliance.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocityRedAlliance.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("TEST", Commands.print("I EXIST"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocityRedAlliance = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityRedAlliance);
    Command driveFieldOrientedAnglularVelocityBlueAlliance = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityBlueAlliance);
    Command driveFieldOrientedAnglularVelocitySim = m_swerveSubsystem.driveFieldOriented(driveAngularVelocitySim);
    Command driveRobotOrientedAngularVelocity  = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

    if(m_swerveSubsystem.isRedAlliance() == true){
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocityRedAlliance);
    }else{
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocityBlueAlliance);
    }


    if (Robot.isSimulation())
    {
      m_driverController.start().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
      DRIVER_B_BUTTON.whileTrue(
          m_swerveSubsystem.pathfindThenFollowPath(
              new Pose2d(
              new Translation2d(5.287, 2.642), 
              Rotation2d.fromDegrees(51.212))));
    } else
    {
      DRIVER_Y_BUTTON.whileTrue(m_swerveSubsystem.centerModulesCommand());
      DRIVER_A_BUTTON.onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
      DRIVER_X_BUTTON.onTrue(Commands.runOnce(m_swerveSubsystem::addFakeVisionReading));
      DRIVER_B_BUTTON.whileTrue(
          m_swerveSubsystem.pathfindThenFollowPath(
              new Pose2d(
              new Translation2d(5.287, 2.642), 
              Rotation2d.fromDegrees(51.212))));

      DRIVER_LEFT_BUMPER.whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());
    }

  }

  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
