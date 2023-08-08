// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // joystick channels
  public static final int RIGHT_JOYSTICK_CHANNEL = 2;
  public static final int LEFT_JOYSTICK_CHANNEL = 0;

  // arm angle positions
  public static final int HOME_ARM_ANGLE = 40; 
  public static final int TRAVELING_ARM_ANGLE_NOT_BLOCKING_CHASSIS_CAM = 295;
  public static final int TOP_SCORING_ANGLE = 207;
  public static final int TOP_TRANSITION_ANGLE = 210;
  public static final int MID_SCORING_ANGLE = 233;
  public static final int BOTTOM_SCORING_ANGLE = 326;
  public static final int SUBSTATION_ANGLE = 279;
  public static final boolean HOME_EXTEND = false;
  public static final boolean TRAVELING_ARM_EXTEND = false;
  public static final boolean TOP_SCORING_EXTEND = true;
  public static final boolean MID_SCORING_EXTEND = true;
  public static final boolean BOTTOM_SCORING_EXTEND = true;
  public static final boolean PICKING_UP_EXTEND = true;
  public static final boolean SUBSTATION_EXTEND = false;
  // arm motor constants
  public static final int ARM_EXTENDED_ALPHA = 116;
  public static final int ARM_EXTENDED_LOWER_LIMIT = 50;
  public static final int ARM_EXTENDED_UPPER_LIMIT = 360;

  public static final int ARM_RETRACTED_ALPHA = 80;
  public static final int ARM_RETRACTED_LOWER_LIMIT = 25;
  public static final int ARM_RETRACTED_UPPER_LIMIT = 335;

  public static final double FB_UPPER_LIMIT_CLOSED = 0.2;
  public static final double FB_LOWER_LIMIT_CLOSED = -0.2;
  public static final double FB_UPPER_LIMIT_OPEN = 0.35;
  public static final double FB_LOWER_LIMIT_OPEN = -0.35;

  public static final double ARM_MOTOR_FF_GAIN = -0.15;
  public static final double ARM_ENCODER_OFFSET = -349;

  // hat constants 
  public static final int HAT_JOYSTICK_TRIM_POSITION = RIGHT_JOYSTICK_CHANNEL;
  public static final int HAT_JOYSTICK_TRIM_ROTATION_ARM = LEFT_JOYSTICK_CHANNEL;
  public static final double HAT_POWER_MOVE = 0.05;
  public static final double HAT_POWER_ROTATE = 0.2;
  // Hat trim target speed is 15 degrees per second
  // One time step is 0.02 seconds
  // 0.3 degrees per time step is our target change when the hat is active
  public static final double HAT_POSE_TARGET_PER_TIME_STEP = -0.3; // negative is raising the arm
  public static final int HAT_POV_MOVE_LEFT = 270;
  public static final int HAT_POV_MOVE_RIGHT = 90;
  public static final int HAT_POV_MOVE_FORWARD = 0;
  public static final int HAT_POV_MOVE_BACK = 180;
  public static final int HAT_POV_ARM_UP = 0;
  public static final int HAT_POV_ARM_DOWN = 180;
  public static final int HAT_POV_ROTATE_LEFT = 270;
  public static final int HAT_POV_ROTATE_RIGHT = 90;
  // absolute enconder for top arm id
  public static final int ARM_ENCODER_ID = 7;

  // arm driving motor id
  public static final int TALON_CHANNEL = 21;

  public static class OperatorConstants {
  public static final int kDriverControllerPort = 0;

  }
  
}
