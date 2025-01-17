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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AbsoluteDriveAdv;
import frc.robot.Subsystems.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Auto Chooser Definitions
  private final SendableChooser<Command> autoChooser;
  // Controller Defitions
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Trigger DRIVER_A_Button = new Trigger(() -> m_driverController.getHID().getAButton());
  private final Trigger DRIVER_B_Button = new Trigger(() -> m_driverController.getHID().getBButton());
  private final Trigger DRIVER_X_Button = new Trigger(() -> m_driverController.getHID().getXButton());
  private final Trigger DRIVER_Y_Button = new Trigger(() -> m_driverController.getHID().getYButton());

  private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  private final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.leftTrigger());
  private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.rightTrigger());
  private final Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.leftBumper());
  private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.rightBumper());


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
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
          OperatorConstants.DRIVER_DEADBAND),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
          OperatorConstants.DRIVER_DEADBAND),
      () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
          OperatorConstants.DRIVER_DEADBAND),
      m_driverController.getHID()::getYButtonPressed,
      m_driverController.getHID()::getAButtonPressed,
      m_driverController.getHID()::getXButtonPressed,
      m_driverController.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX() * OperatorConstants.ROTATION_SPEED)
      .deadband(OperatorConstants.DRIVER_DEADBAND)
      .scaleTranslation(OperatorConstants.TRANSLATION_SPEED)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
      m_driverController::getRightX,
      m_driverController::getRightY)
      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = m_swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // Simulation Drive CMDs
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
      .deadband(OperatorConstants.DRIVER_DEADBAND)
      .scaleTranslation(OperatorConstants.TRANSLATION_SPEED)
      .allianceRelativeControl(true);

  // Derive the heading axis with math
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_driverController.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              m_driverController.getRawAxis(
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
    // Odometry Reset
      m_driverController.start().onTrue(Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    // Add Fake Vision Reading
      DRIVER_B_Button.onTrue(Commands.runOnce(m_swerveSubsystem :: addFakeVisionReading));

    } else {
      // Zero Gyro
      DRIVER_A_Button.onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
      // Drive to Pose
      DRIVER_B_Button.whileTrue(
          m_swerveSubsystem.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // X Wheels
      DRIVER_X_Button.whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem).repeatedly());
      // Drive 1 meter forward
      DRIVER_Y_Button.whileTrue(m_swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
      // SysID CMD
      DRIVER_POV_UP.whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());

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
