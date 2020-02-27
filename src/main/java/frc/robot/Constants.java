/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Encoder DIO Ports
    public static int encoder_LeftA  = 8;
    public static int encoder_RightA = 6;
    public static int encoder_LeftB  = 9;
    public static int encoder_RightB = 7;

    //constants E4T encoders CPR = 360 x 4 = 1440 pulses /revolution 
    public static double ENCODER_PPR = 360.0*4;//360 CPR * 4pulses/rev

    // Need to adjust Wheel Diameter for new robot, 7.5" ?
    //public static double WHEEL_DIAM = 6.0;//inches Pratice Bot
    public static double WHEEL_DIAM = 7.5;//inches 2020 Bot
    public static double MWHEEL_DIAM = Units.inchesToMeters(WHEEL_DIAM); 

    public static double DIST_PER_PULSE = ((WHEEL_DIAM*Math.PI)/ENCODER_PPR)*1.14;
    public static double MDIST_PER_PULSE = Units.inchesToMeters(DIST_PER_PULSE);
    public static double DIST_ADJUSTMENT = 3.55; //This might need to be tweeked for new robot

    //Need to adjust Track Width for the new robot, 19.25" ?
    //public static double TRACK_WIDTH = 23.0; //inches Practice Bot
    public static double TRACK_WIDTH = 19.25; //inches 2020 Bot
    public static double MTRACK_WIDTH = Units.inchesToMeters(TRACK_WIDTH);

    public static double MAX_ROTATION = 2.0*Math.PI;
    public static double MAX_SPEED = 3.0;

    //Drive Train Motor CAN Bus Addresses Front Motors are Masters
    public static int drive_LeftFrontMotor  = 10;
    public static int drive_RightFrontMotor = 20;
    public static int drive_LeftRearMotor   = 11;
    public static int drive_RightRearMotor  = 21;

    //CAN Bus Address for Spinner Motor (Used to spin the color wheel)
    public static final int spinner_CentralMotor = 30;

    //CAN Bus Addresses for the Hopper Motors, Motor B is the Master
    public static final int hopper_motorA = 31;
    public static final int hopper_motorB = 32;

    //Can Bus Address for the Ball Shotter Motors
    public static final int shooter_motorA = 33;
    public static final int shooter_motorB = 34;

    //Can Bus Address for the Ball Intake Motor
    public static final int intake_motor =35;
    
    //Can Bus and Timeout for the Gyroscope
    public static final int PIGEON_IMU = 40;
    public static final int PIGEON_TIMEOUT = 38; //ms

    //Used for setting the lower end JoyStick Deadzone
    public static final double JOYSTICK_MIN = 0.2;  
}
