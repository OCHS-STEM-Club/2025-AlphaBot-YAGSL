// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsytem extends SubsystemBase {
  private TalonFXS endEffectorMotor;
  private TalonFXSConfiguration endEffectorConfiguration;

  /** Creates a new EndEffectorSubsytem. */
  public EndEffectorSubsytem() {
    endEffectorMotor = new TalonFXS(EndEffectorConstants.EndEffectorMotorID);

    endEffectorConfiguration = new TalonFXSConfiguration()
                                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake));
    endEffectorMotor.getConfigurator().apply(endEffectorConfiguration);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void endEffectorMotorOn() {
    endEffectorMotor.set(Constants.EndEffectorConstants.kEndEffectorOnSpeed);
  }
  public void endEffectorMotorOff() {
    endEffectorMotor.set(0);
  }
  public void endEffectorMotorReverse() {
    endEffectorMotor.set(Constants.EndEffectorConstants.kEndEffectorReverseSpeed);
  }
}
