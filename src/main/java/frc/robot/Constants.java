package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.631;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.632;

  public static final double FRONT_LEFT_MODULE_STEER_OFFSET  = -2.559;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET  = -5.074;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET   = -0.514;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -4.929;

  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 16;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 15;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 9;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
  
  //public static final double MOUNT_PITCH = -5.53711;
  public static final double MOUNT_PITCH = 0;
  public static final double MOUNT_YAW = 0;
  public static final double MOUNT_ROLL = 0;

}
