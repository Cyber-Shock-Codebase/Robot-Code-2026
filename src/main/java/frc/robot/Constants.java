// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class intakeConstants
  {

    public static final double IN_SPEED_NORM = 2500;
    public static final double IN_SPEED_Fast = 5000;
    public static final double OUT_SPEED_NORM = -2500;
    public static final double OUT_SPEED_FAST = -5000;
    public static final int MOTOR_ID = 9;
  }

  public static class ShooterConstants
  {

    public static final int FEEDER_MOTOR_ID = 2;
    public static final double feederSpeedNorm = 0.5;
    public static final double flywheelTolerance = 100; // RPM tolerance for feeder to activate
    public static final Angle headingTolerance = Degrees.of(1); // degrees tolerance for feeder to activate
    public static final int SHOOTER_MOTOR0_ID = 3;
    // public static final int SHOOTER_MOTOR1_ID = 4;
    public static final int SHOOTER_MOTOR2_ID = 5;
    public static final int SHOOTER_MOTOR3_ID = 6;
    public static final int SHOOTER_MOTOR4_ID = 7;
    // public static final int SHOOTER_MOTOR5_ID = 8;
    public static final AngularVelocity minSpeed = RPM.of(100); // RPM, minimum speed to shoot
    public static final AngularVelocity maxSpeed = RPM.of(9000); // RPM, maximum speed to shoot
  }


    
}

