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
    public static final int encoder_LeftA = 8;
    public static final int encoder_RightA = 6;
    public static final int encoder_LeftB = 9;
    public static final int encoder_RightB = 7;

    //constants E4T encoders CPR = 360 x 4 = 1440 pulses /revolution 
    public static final double ENCODER_PPR = 360.0*4;//360 CPR * 4pulses/rev
    public static final double  WHEEL_DIAM = 6.0;//inches
    public static final double MWHEEL_DIAM = Units.inchesToMeters(WHEEL_DIAM); 
    public static final double DIST_PER_PULSE = ((WHEEL_DIAM*Math.PI)/ENCODER_PPR)*1.14;//*(2/7);
    public static final double MDIST_PER_PULSE = Units.inchesToMeters(DIST_PER_PULSE);
    
    public static final double TRACK_WIDTH = 23.0; //inches
    public static final double MTRACK_WIDTH = Units.inchesToMeters(TRACK_WIDTH);
    public static final double MAX_SPEED = 3.0;
    public static final double MAX_ROTATION = 2.0*Math.PI;


    public static final int drive_LeftFrontMotor = 10;
    public static final int drive_RightFrontMotor  = 20;
    public static final int drive_LeftRearMotor = 11;
    public static final int drive_RightRearMotor = 21;

    public static final int spinner_CentralMotor = 30;
    

}
