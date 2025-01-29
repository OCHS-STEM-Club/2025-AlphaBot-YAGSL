// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import swervelib.math.Matter;

public final class Constants {
  // TODO:Make sure to update this information
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 5.95; // In meter per second


  public static final class AutoConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DriveConstants {

    
  }

  public static class OperatorConstants {
    // Driver Controller Port
    public static final int kDriverControllerPort = 0;
    // Joystick Deadband
    public static final double DRIVER_DEADBAND = 0.1;
    // Rotation Multiplier
    public static final double ROTATION_SPEED = 0.75;
    // Translation Speed
    public static final double TRANSLATION_SPEED = 0.8;
    

  }

  public static class VisionConstants {
    // Camera Definitions
    public static final String CAMERA_NAME = "Center";
    // Camera to Robot Transform
    public static final Transform3d BACK_LEFT_CAM_TO_CENTER = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-14.063), // X value from center of robot REMEMBER THAT IT IS RELATIVE TO THE PIGEON DIRECTION
        Units.inchesToMeters(14.063), // Y value from center of robot REMEMBER THAT IT IS RELATIVE TO THE PIGEON DIRECTION
        Units.inchesToMeters(7.25)), // Z value from center of robot REMEMBER THAT IT IS RELATIVE TO THE PIGEON DIRECTION
        //TODO: Make sure this is in radians
        new Rotation3d(0,0,15)); // Rotation of the camera relative to the robot
    // Standard Deviations for Pose Estimation
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }


   public static final class ElevatorConstants {
    public static final int ELEVATOR_LEFT_CAN_ID = 18;
    public static final int ELEVATOR_RIGHT_CAN_ID = 19;

    // Gear ratio
    public static final double ELEVATOR_GEAR_RATIO = 5;
    public static final Distance ELEVATOR_SPROCKET_DIAMETER = Inches.of(1.888);

    public static final Distance FEET_PER_ROTATION = ELEVATOR_SPROCKET_DIAMETER.times(Math.PI).div(ELEVATOR_GEAR_RATIO);//TODO:Fix this to work in feet instad of meters

    // Maximum and minimum extension of the elevator, in meters
    public static final Distance MAX_HEIGHT = Inches.of(88.5); 
    public static final Distance MIN_HEIGHT = Inches.of(0); 

    // Current limit of either motor
    public static final Current CURRENT_LIMIT = Amps.of(20);//TODO:Find Elevator Current Limit from derek

    // Left Motor PID Values
    public static final double LEFT_KG = 0;
    public static final double LEFT_KS = 0;
    public static final double LEFT_KV = 0;
    public static final double LEFT_KP = 1;
    public static final double LEFT_KI = 0;
    public static final double LEFT_KD = 0;

    // Right Motor PID Values
    public static final double RIGHT_KG = 0;
    public static final double RIGHT_KS = 0;
    public static final double RIGHT_KV = 0;
    public static final double RIGHT_KP = 1;
    public static final double RIGHT_KI = 0;
    public static final double RIGHT_KD = 0;


    public static final MotorOutputConfigs RIGHT_MOTOR_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    // Elevator Positions
    public static final Distance STATION_POSITION = Inches.of(0);
    public static final Distance L1_POSITION = Inches.of(20);
    public static final Distance L2_POSITION = Inches.of(30);
    public static final Distance L3_POSITION = Inches.of(0);
    public static final Distance L4_POSITION = Inches.of(0);
    public static final Distance ALGAE_POSITION = Inches.of(0);

    // Maximum velocity of the Motors, in Rotations per Second
    public static final double MAX_VELOCITY_RPS = 10; //TODO:Change this

    // Maximum acceleration of the Motors, in Rotations per Second
    public static final double MAX_ACCEL_RPS = 10; //TODO:Change this

    public static final Voltage MAX_VOLTS = Volts.of(8);//TODO:Check with Derek about Max Volts

    // Creates new set states for the Trapezoid Profile
    public static final TrapezoidProfile.State STOW_GOAL = new TrapezoidProfile.State(MIN_HEIGHT.magnitude(), 10);
    public static final TrapezoidProfile.State STATION_GOAL = new TrapezoidProfile.State(STATION_POSITION.magnitude(), 10);
    public static final TrapezoidProfile.State L1_GOAL = new TrapezoidProfile.State(L1_POSITION.magnitude(), 10);
    public static final TrapezoidProfile.State L2_GOAL = new TrapezoidProfile.State(L2_POSITION.magnitude(), 10);
    public static final TrapezoidProfile.State L3_GOAL = new TrapezoidProfile.State(L3_POSITION.magnitude(), 10);
    public static final TrapezoidProfile.State L4_GOAL = new TrapezoidProfile.State(L4_POSITION.magnitude(), 10);
  }
}
