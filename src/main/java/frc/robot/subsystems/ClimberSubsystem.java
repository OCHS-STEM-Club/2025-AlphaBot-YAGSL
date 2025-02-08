// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private TalonFX climberMotor;
  private Servo climberServo;

  private TalonFXConfiguration climberConfigs;

  public ClimberSubsystem() {
    climberMotor = new TalonFX(24);
    climberServo = new Servo(0);

    climberConfigs = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive));

    climberMotor.getConfigurator().apply(climberConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void servoUp() {
    climberServo.setSpeed(.5);
  }

  public void servoDown() {
    climberServo.setSpeed(.5);
  }
  
  public void servoStop() {
    climberServo.setSpeed(0);
  }

  public void climberMotorUp() {
    climberMotor.set(.25);
  }

  public void climberMotorDown() {
    climberMotor.set(-.25);
  }

  public void climberMotorStop() {
    climberMotor.set(0);
  }

  @AutoLogOutput(key = "Climber/Climber Servo")
  public double getClimberServo(){
    return climberServo.get();
  }

  @AutoLogOutput(key = "Climber/Climber Motor")
  public double getClimberMotor() {
    return climberMotor.get();
  }
}
