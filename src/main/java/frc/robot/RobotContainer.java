// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Auto Chooser Definitions
  private final SendableChooser<Command> autoChooser;
  // Controller Defitions
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // Subsystem Defintions
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in
  // configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_swerveSubsystem,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
          OperatorConstants.DRIVER_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
          OperatorConstants.DRIVER_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
          OperatorConstants.DRIVER_DEADBAND),
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getXButtonPressed,
      driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> driverXbox.getLeftY(),
      () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * OperatorConstants.ROTATION_SPEED)
      .deadband(OperatorConstants.DRIVER_DEADBAND)
      .scaleTranslation(OperatorConstants.TRANSLATION_SPEED)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
      driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = m_swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // Simulation Drive CMDs
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DRIVER_DEADBAND)
      .scaleTranslation(OperatorConstants.TRANSLATION_SPEED)
      .allianceRelativeControl(true);

  // Derive the heading axis with math
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = m_swerveSubsystem.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = m_swerveSubsystem.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = m_swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Auto Chooser Intitialization
    autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the trigger bindings
    configureBindings();
    // Silence Warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    // PID Feedforward values
    m_swerveSubsystem.replaceSwerveModuleFeedforward(0.18785, 2.0895, 0.18022);
    // Put Auto Chooser on dashboard
    SmartDashboard.putData("Autos", autoChooser);
  }

  // Controller Binding method
  private void configureBindings() {
    // Set Drive CMD
    m_swerveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

    if (Robot.isSimulation()) {
      driverXbox.start()
          .onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());

    } else {
      // Zero Gyro
      driverXbox.a().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
      // Drive to Pose
      driverXbox.b().whileTrue(
          m_swerveSubsystem.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // X Wheels
      driverXbox.x().whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());
      // Drive 1 meter forward
      driverXbox.y().whileTrue(m_swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
      // SysID CMD
      driverXbox.povUp().whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());

    }

  }

  // Auto Return CMD
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Method to set idle state of robot
  public void setMotorBrake(boolean brake) {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
