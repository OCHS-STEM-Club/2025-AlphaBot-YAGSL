// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGroundIntakeConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
  private TalonFX intakePivotMotor; 
  private TalonFX intakeMotor;
  private CANcoder pivotCANCoder;


  private TalonFXConfiguration intakePivotMotorConfiguration;
  private TalonFXConfiguration intakeMotorConfiguration;

  private PositionVoltage m_positionRequest;

  private DigitalInput intakeSensor;
  
  

  /** Creates a new CoralGroundIntake. */
  public CoralGroundIntakeSubsystem() {
    intakePivotMotor = new TalonFX(CoralGroundIntakeConstants.kCoralGroundIntakePivotID);
    intakeMotor = new TalonFX(CoralGroundIntakeConstants.kCoralGroundIntakeID);
    pivotCANCoder = new CANcoder(CoralGroundIntakeConstants.kCANCoderID);
    

    intakePivotMotorConfiguration = new TalonFXConfiguration()
                                    .withSlot0(new Slot0Configs()
                                              .withKP(CoralGroundIntakeConstants.kIntakeP)
                                              .withKI(CoralGroundIntakeConstants.kIntakeI)
                                              .withKD(CoralGroundIntakeConstants.kIntakeD))
                                    .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                                    .withFeedback(new FeedbackConfigs()
                                                      .withFeedbackRemoteSensorID(pivotCANCoder.getDeviceID())
                                                      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));


    intakePivotMotor.getConfigurator().apply(intakePivotMotorConfiguration);
    
    intakeMotorConfiguration = new TalonFXConfiguration()
                                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake));
    intakeMotor.getConfigurator().apply(intakeMotorConfiguration);

   m_positionRequest = new PositionVoltage(0).withSlot(0);



    intakePivotMotor.setControl(m_positionRequest.withPosition(0));


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void coralGroundIntake() {
    intakeMotor.set(CoralGroundIntakeConstants.kIntakeSpeed);
  }
  
  public void coralGroundOuttake() {
    intakeMotor.set(-1 * CoralGroundIntakeConstants.kIntakeSpeed);
  }

  public void coralGroundOff() {
    intakeMotor.set(0);
  }

  public void intakePivotDown() {
    intakePivotMotor.set(-1 * CoralGroundIntakeConstants.kIntakePivotSpeed);
  }

  public void intakePivotUp() {
    intakePivotMotor.set(CoralGroundIntakeConstants.kIntakePivotSpeed);
  }

  public void coralGroundIntakeDownSetPoint() {
    intakePivotMotor.setControl(m_positionRequest.withPosition(0));
  }

  public void coralGroundIntakeUpSetPoint() {
    intakePivotMotor.setControl(m_positionRequest.withPosition(0));
  }

  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

}
