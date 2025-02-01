// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

  public static class CoralGroundIntakeConstants {
    public static final int kCoralGroundIntakeID = 21;
    public static final int kCoralGroundIntakePivotID = 20;
    public static final int kCANCoderID = 22; 
    
    public static final double kIntakeP = 0;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;

    public static final double kIntakeSpeed = 0.25;
    public static final double kIntakePivotSpeed = 0;
  }
}
