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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.608; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.608;


    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(133.93); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(248.813); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(94.75); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(67.324); // FIXME Measure and set back right steer offset
    
    
    public static final double kHoodMinValue = 0.15;
    public static final double kHoodMaxValue = 0.47;

    public static final double rotationModifier = 0.5;

    public static final double ball1Xpos = -0.658;
    public static final double ball1Ypos = -3.83;
    public static final double ball2Xpos = -3.174;
    public static final double ball2Ypos = -2.243;
    public static final double ball3Xpos = -3.287;
    public static final double ball3Ypos = 2.074;
    public static final double ball4Xpos = -7.165;
    public static final double ball4Ypos = -2.99;

    public static final double pos1X = -0.686;
    public static final double pos1Y = -2.753;
    public static final double pos1theta = -88.5;

    public static final double pos2X = -1.959;
    public static final double pos2Y = -1.702;
    public static final double pos2theta = -156;

    public static final double pos3X = -2.505;
    public static final double pos3Y = 1.332;
    public static final double pos3theta = 136;

    public static final double kickItWaitTime = 1.2;
    public static final double shooterReverseTime = 0.25;
    public static final double shooterAutoTime = 1.0;
}
