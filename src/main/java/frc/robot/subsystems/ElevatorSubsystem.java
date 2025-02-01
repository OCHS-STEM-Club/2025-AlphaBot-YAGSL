// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Creating the Motor Controller objects
  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  // Creating the Motor Controller Configuration objects
  private final TalonFXConfiguration elevatorLeftConfig;
  private final TalonFXConfiguration elevatorRightConfig;
  private final Follower elevatorFollower;
  
   // Trapezoid Profile for the Elevator
  private TrapezoidProfile elevatorProfile;
  private TrapezoidProfile.State currentElevatorState;
  private PositionVoltage elevatorRequest;
  private VoltageOut voltageRequest;

  private final Mechanism2d elevator2d;
  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorLigament;

  private final SendableChooser<TrapezoidProfile.State> elevatorChooser;

  private final SysIdRoutine sysIdRoutine;

  public ElevatorSubsystem() {
    
    elevatorLeftMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_CAN_ID);
    elevatorRightMotor = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_CAN_ID);

    elevatorFollower = new Follower(elevatorLeftMotor.getDeviceID(), false);
    elevatorRightMotor.setControl(elevatorFollower);

    elevatorLeftConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
                  .withKG(ElevatorConstants.LEFT_KG)
                  .withKS(ElevatorConstants.LEFT_KS)
                  .withKV(ElevatorConstants.LEFT_KV)
                  .withKI(ElevatorConstants.LEFT_KP)
                  .withKD(ElevatorConstants.LEFT_KI)
                  .withKD(ElevatorConstants.LEFT_KD))

                  .withCurrentLimits( new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
                                    .withSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimitEnable(true))

                  .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                              .withReverseSoftLimitThreshold(ElevatorConstants.MIN_HEIGHT.magnitude() / ElevatorConstants.FEET_PER_ROTATION.magnitude())
                                              .withReverseSoftLimitEnable(true)
                                              .withForwardSoftLimitThreshold(ElevatorConstants.MAX_HEIGHT.magnitude() / ElevatorConstants.FEET_PER_ROTATION.magnitude())
                                              .withForwardSoftLimitEnable(true))

                  .withMotorOutput(new MotorOutputConfigs()
                                  .withInverted(InvertedValue.CounterClockwise_Positive)
                                  .withNeutralMode(NeutralModeValue.Brake));

      elevatorLeftConfig.Voltage.withPeakForwardVoltage(ElevatorConstants.MAX_VOLTS.magnitude())
      .withPeakReverseVoltage(ElevatorConstants.MAX_VOLTS.magnitude());

      elevatorRightConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
                  .withKG(ElevatorConstants.RIGHT_KG)
                  .withKS(ElevatorConstants.RIGHT_KS)
                  .withKV(ElevatorConstants.RIGHT_KV)
                  .withKI(ElevatorConstants.RIGHT_KP)
                  .withKD(ElevatorConstants.RIGHT_KI)
                  .withKD(ElevatorConstants.RIGHT_KD))

                  .withCurrentLimits( new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
                                    .withSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
                                    .withStatorCurrentLimitEnable(true)
                                    .withSupplyCurrentLimitEnable(true))

                  .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                              .withReverseSoftLimitThreshold(ElevatorConstants.MIN_HEIGHT.magnitude() / ElevatorConstants.FEET_PER_ROTATION.magnitude())
                                              .withReverseSoftLimitEnable(true)
                                              .withForwardSoftLimitThreshold(ElevatorConstants.MAX_HEIGHT.magnitude() / ElevatorConstants.FEET_PER_ROTATION.magnitude())
                                              .withForwardSoftLimitEnable(true))

                  .withMotorOutput(new MotorOutputConfigs()
                                  .withInverted(InvertedValue.CounterClockwise_Positive)
                                  .withNeutralMode(NeutralModeValue.Brake));

      elevatorRightConfig.Voltage.withPeakForwardVoltage(ElevatorConstants.MAX_VOLTS.magnitude())
        .withPeakReverseVoltage(ElevatorConstants.MAX_VOLTS.magnitude());
  
    
    elevatorRequest = new PositionVoltage(0).withSlot(0); //TODO:Check what this does
    voltageRequest = new VoltageOut(Volts.of(0)); //TODO:Check what this does

    currentElevatorState = new TrapezoidProfile.State();

    elevatorLeftMotor.getConfigurator().apply(elevatorLeftConfig);
    elevatorRightMotor.getConfigurator().apply(elevatorRightConfig);

    elevatorLeftMotor.setPosition(Rotations.of(ElevatorConstants.MIN_HEIGHT.magnitude() / ElevatorConstants.FEET_PER_ROTATION.magnitude()));

    elevatorProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY_RPS, ElevatorConstants.MAX_ACCEL_RPS));

    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(1).per(Second),
        Volts.of(4),
        Seconds.of(1),
        (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())), 
      new SysIdRoutine.Mechanism(
        output -> elevatorLeftMotor.setControl(voltageRequest.withOutput(output)),
        null,
        this));


  elevator2d = new Mechanism2d(1, 1);
  elevatorRoot = elevator2d.getRoot("Base", 0.5, 0.5);

  elevatorLigament = elevatorRoot.append(new MechanismLigament2d("ElevatorExt", 1, 90));

  elevatorChooser = new SendableChooser<TrapezoidProfile.State>();
  elevatorChooser.addOption("-----L4-----", ElevatorConstants.L4_GOAL);
  elevatorChooser.addOption("-----L3-----", ElevatorConstants.L3_GOAL);
  elevatorChooser.addOption("-----L2-----", ElevatorConstants.L2_GOAL);
  elevatorChooser.addOption("-----L1-----", ElevatorConstants.L1_GOAL);
  elevatorChooser.setDefaultOption("Stow", ElevatorConstants.STOW_GOAL);
  elevatorChooser.addOption("Coral Station", ElevatorConstants.STATION_GOAL);


  SmartDashboard.putData("Elevator", this);
  SmartDashboard.putData("Elevator/Elevator2d", elevator2d);
  SmartDashboard.putData("Elevator/ElevatorChooser", elevatorChooser);
  SmartDashboard.putNumber("Elevator/ElevatorSetPoint", currentElevatorState.position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   @AutoLogOutput
  private Angle getElevatorRotations() {
    return elevatorLeftMotor.getPosition().refresh().getValue();
  }
  @AutoLogOutput
  private Distance getElevatorHeight() {
    return ElevatorConstants.FEET_PER_ROTATION.times(getElevatorRotations().in(Rotations));
  }
  @AutoLogOutput
  private LinearVelocity getElevatorVelocity() {
    return FeetPerSecond.of(elevatorLeftMotor.getVelocity().refresh().getValueAsDouble() * ElevatorConstants.FEET_PER_ROTATION.magnitude());
  }
  @AutoLogOutput
  private TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getElevatorHeight().magnitude(), getElevatorVelocity().magnitude());
  }
  
  public TrapezoidProfile.State getSelectedState() {
    return elevatorChooser.getSelected();
  }
  @AutoLogOutput
  public String getSelectedString() {
    // Systae
    return elevatorChooser.getSelected().toString();
  }

  public void stopElevator() {
    elevatorLeftMotor.stopMotor();
  }

  public void applyElevatorProfile(TrapezoidProfile.State goal) {
    currentElevatorState = elevatorProfile.calculate(.20, currentElevatorState, goal);
    elevatorRequest.Position = currentElevatorState.position;
    elevatorRequest.Velocity = currentElevatorState.velocity;

    elevatorLeftMotor.setControl(elevatorRequest);
  }


  public void setElevatorPosition(double rotations) {
    elevatorLeftMotor.setControl(elevatorRequest.withPosition(rotations));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction.kForward);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction.kForward);
  }

  public Command setElevatorPositionCommand(TrapezoidProfile.State goalState) {
    final double deltaTime = 0.02;
    final Timer timer = new Timer();
    System.out.println("elevator go");
    
    return runOnce(timer::restart)
    .andThen(run(() -> {
      currentElevatorState = elevatorProfile.calculate(deltaTime, getCurrentState(), goalState);
      elevatorLeftMotor.setControl(elevatorRequest.withPosition(currentElevatorState.position));}))
      .until(() -> timer.hasElapsed(elevatorProfile.totalTime()));
     
  }

  @AutoLogOutput
  public double getElevatorLeftMotorVelocity(){
    return elevatorLeftMotor.get();

  }
}
