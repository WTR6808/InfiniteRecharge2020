/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class LimeLight {
    private static final double TX_MAX = 27.0/2.5;//54.0;
    private static final double TY_MAX = 20.5/2.5;//41.0;
  
    private static final double X_TOLERANCE = 1.0;
    private static final double Y_TOLERANCE = 0.25;
//    private static final double LIMIT = 1.0;
  
    private static double  leftSpeed   = 0.0;
    private static double  rightSpeed  = 0.0;
    private static boolean validTarget = false;
  
    private static int targetFailureCount =  0;
    private static final int MAX_FAILURES = 10; //200mSec delay
  
    public boolean mode = false;
	
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ta = table.getEntry("ta");
   // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private double driveCommand = 0.0;
    private double steerCommand = 0.0;
    private boolean takeSnapShots = false;

    private static final double STEER_K           =  0.037;
    private static final double DRIVE_K           =  0.048;//0.01;
    //private static final double DESIRED_TA        = 19.5;
    private static final double MAX_DRIVE         =  0.85; //0.75 PracticeBot
	  private static final double MIN_DRIVE         =  0.50; //0.40 PracticeBot
	  private static final double MIN_STEER         =  0.30; //0.30 PracticeBot
	  private static final double MIN_STEER_STOPPED =  0.45; //0.45 PracticeBot
    //private static final double TA_TOLERANCE      =  0.1;
    private static final double STEER_CORRECT     =  0.0; //-0.2;

    public double getSteerCommand(){
      return steerCommand;
    }

    public double getDriveCommand(){
      return driveCommand;
    }

    public double getTA(){
      return ta.getDouble(0.0);
    }
  
    public double getTX(){
      return tx.getDouble(0.0);
    }
                                  
    public double getTY(){
      return ty.getDouble(0.0);
    }

    public boolean getTV(){
      return tv.getDouble(0)%2 != 0;
    }
  
    public void toggleSnapShot(){
      takeSnapShots = !takeSnapShots;
      table.getEntry("snapshot").setNumber(takeSnapShots?1:0);
    }
    
    public void setLEDOn(boolean on){
      table.getEntry("ledMode").setNumber(on?3:1);
    }

    public void setCameraMode(boolean drive){
      //drive = true will set mode to stream, false to vision
      table.getEntry("camMode").setNumber(drive?1:0);
      //Turn LEDs On/Off based on camMode. stream = LEDs Off, vision = LEDs On
      setLEDOn(!drive);
  
      //Start speed at 0.  May need to adjust based on testing
      //leftSpeed  = 0.2;
      //rightSpeed = 0.2;
      
	    //Reset the no target counter
	    targetFailureCount = 0;
    }
  
    public double getLeftSpeed(){
      return leftSpeed;
    }

    public double getRightSpeed(){
      return rightSpeed;
    }
  
    public boolean withinTolerance(){
	    //Test for valid target
	    if (validTarget){
		    //If tracking a target test X and Y within tolerances
		    return ((Math.abs(getTX()) < X_TOLERANCE) && (Math.abs(getTY()) < Y_TOLERANCE));
	    }else{
	      return false;
	    }
    }

    //New version of calcSpeeds.  The old one looks like the speeds will never slow down.
    public boolean calcSpeeds(){
      double headingAdj;
      double distanceAdj;
      double greater;
      double lesser;
      double ratio = 1.0;
	    validTarget = getTV();
      if (validTarget){
		    targetFailureCount = 0;
        headingAdj  = (getTX()/TX_MAX)*1.1; //Normalized from -1.0 to 1.0
        distanceAdj = getTY()/TY_MAX; //Normalized from -1.0 to 1.0
        SmartDashboard.putString("Valid Target", "Target Acquired");
      }else{
        //Alert the operator to no target and set speed adjustments to 0
        SmartDashboard.putString("Valid Target", "No Valid Target");
		    targetFailureCount++;
        headingAdj = 0.0;
        distanceAdj = 0.0;
      }
      //Calculate Ratio to Normalize left & right speeds to -1.0 to 1.0
      greater = Math.max(Math.abs(headingAdj),Math.abs(distanceAdj));
      lesser  = Math.min(Math.abs(headingAdj),Math.abs(distanceAdj));
      if(greater > 0.0) {
        ratio = (lesser/greater) + 1.0;
      } else {
        ratio = 1.0;
      }

      headingAdj  = headingAdj/ratio;
      distanceAdj = distanceAdj/ratio;
      SmartDashboard.putNumber("Heading Adj", headingAdj);
      SmartDashboard.putNumber("Distance Adj", distanceAdj);

      //Calculate the speeds
      leftSpeed  = (distanceAdj-headingAdj);
      rightSpeed = (distanceAdj+headingAdj);
      leftSpeed  = (Math.abs(leftSpeed)  < 0.5)?Math.signum((leftSpeed ))*.4:leftSpeed;
      rightSpeed = (Math.abs(rightSpeed) < 0.5)?Math.signum((rightSpeed))*.4:rightSpeed;
	  
      return (validTarget || (targetFailureCount <= MAX_FAILURES));
  }
  public boolean getBool(){
    if(mode){
      mode = false;
      return false;
    }else{
      mode = true;
      return true;
    }
	  //Could also just have done
	  //mode = !mode;
	  //return mode;
  }
  
  public boolean calcDoubleS(){
    validTarget = getTV();
    if (validTarget){
	    targetFailureCount = 0;
		  //May need to set some minimum steer and drive values
      steerCommand = getTX() * STEER_K  + STEER_CORRECT;
      //driveCommand = (DESIRED_TA - getTA()) * DRIVE_K;
      driveCommand = getTY() * DRIVE_K;
		  normalizeDrive();
      SmartDashboard.putString("Valid Target", "Target Acquired");
    }else{
      //Alert the operator to no target and set speed adjustments to 0
      SmartDashboard.putString("Valid Target", "No Valid Target");
	    targetFailureCount++;
      driveCommand = 0.0;
      steerCommand = 0.0;
    }
    //limit the drive speed
    //if (driveCommand < 0.4){
    //  driveCommand = 0.4;
    //}
	  //driveCommand = ((Math.abs(driveCommand) >= MAX_DRIVE)?Math.signum(driveCommand)*MAX_DRIVE:driveCommand);
    return (validTarget || (targetFailureCount <= MAX_FAILURES));
  }
  
  private void normalizeDrive(){
	  //Minimum steering value changes based on robot moving or stationary
	  double minSteer = 0.0;
	  
	  //Test if within distance tolerances
	  if (Math.abs(getTY()) > Y_TOLERANCE){
	    //Normalize to be between min and max driving speeds
		  driveCommand=(Math.abs(driveCommand)<MIN_DRIVE)?driveCommand+Math.signum(driveCommand)*MIN_DRIVE:driveCommand;
		  driveCommand=(Math.abs(driveCommand)>MAX_DRIVE)?Math.signum(driveCommand)*MAX_DRIVE:driveCommand;
		
		  //Robot is moving, change minimum steering value
		  minSteer = MIN_STEER;
	  }else{
		  driveCommand=0.0;
		  minSteer = MIN_STEER_STOPPED;
	  }		
	  
	  //Test if within turning rate tolerances
	  if (Math.abs(getTX()) > X_TOLERANCE){
		  //At target, not moving, use minimum for stationary
		  steerCommand=(Math.abs(steerCommand)<minSteer)?steerCommand+Math.signum(steerCommand)*minSteer:steerCommand;
	  }else{
		  steerCommand=0.0;
	  }
  }
  
  public boolean withinDoubleSTolerance(){
	  //Test for valid target
	  return withinTolerance();
	  //if (validTarget){
	  //  //If tracking a target test TA within tolerances
  	//  return (Math.abs(DESIRED_TA - getTA()) < TA_TOLERANCE);
	  //}else{
	  //  return false;
	  //}
  }
}