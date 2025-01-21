// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 5.95; // In meter per second


  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DriveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // In seconds
    
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
}
