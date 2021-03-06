/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.classes.LimeLight;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveTrain extends SubsystemBase {

  private WPI_VictorSPX leftMaster = new WPI_VictorSPX(Constants.drive_LeftFrontMotor);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.drive_LeftRearMotor);
  private WPI_VictorSPX rightMaster = new WPI_VictorSPX(Constants.drive_RightFrontMotor);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.drive_RightRearMotor);
  private DifferentialDrive DriveTrain = new DifferentialDrive(leftMaster, rightMaster);
  
  private Encoder leftEncoder = new Encoder(Constants.encoder_LeftA, Constants.encoder_LeftB, true, CounterBase.EncodingType.k2X);
  private Encoder rightEncoder = new Encoder(Constants.encoder_RightA, Constants.encoder_RightB, false, CounterBase.EncodingType.k2X);

  private PigeonIMU pIMU = new PigeonIMU(Constants.PIGEON_IMU);
  private int direction = 1;
  private boolean intakeIsFront = true;
  
  //                                                         P   I  D
  private final PIDController m_leftPID = new PIDController (.30, 0, 0);
  private final PIDController m_rightPID = new PIDController(.30, 0, 0);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.MTRACK_WIDTH);

  //Create a LimeLight Camera object
  private final LimeLight limeLight = new LimeLight();
  private boolean limeLightDriverMode = true;
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    //For Testing a new Drivetrain (each motor independantly), comment out the
    //  follow(...) commands to separate control of the motors.
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // Winged Hussars May need Work

    //For tuning a new DriveTrain, use setInverted so that the controller blinks
    //  Green for forward motion, and Red for reverse motion.  Motors on the same
    //  side should have the same inversion setting
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);

    //Did not test if using InvertType.FollowMaster for inversion works.  Do this testing
    //  when time permits instead of forcibly setting slave inversion same as master's

    //leftSlave.setInverted(InvertType.FollowMaster);
    //rightSlave.setInverted(InvertType.FollowMaster);

    //For tuning motors independantly, comment this out.  After using motors paired with
    //  either ArcadeDrive or TankDrive, uncomment this line if one side runs opposite of
    //  the other causing rotation instead of forward/reverse motion.
    DriveTrain.setRightSideInverted(false);

    //Control the Ramp Rate of the Motor Speed, Ramp Rate should not be used with PID Control?
    rightMaster.configOpenloopRamp(Constants.DRIVETRAIN_RAMP_RATE);
    rightSlave.configOpenloopRamp(Constants.DRIVETRAIN_RAMP_RATE);
    leftMaster.configOpenloopRamp(Constants.DRIVETRAIN_RAMP_RATE);
    leftSlave.configOpenloopRamp(Constants.DRIVETRAIN_RAMP_RATE);

    // Program the Encoders
    leftEncoder.setDistancePerPulse(Constants.LEFT_MDIST_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.RIGHT_MDIST_PER_PULSE);
    resetDistance();
    
    //Initialize the LimeLight to Driver Mode
    setDriverMode();

    pIMU.configFactoryDefault();
    pIMU.setFusedHeading(0.0, Constants.PIGEON_TIMEOUT);
  }
  /*End Creation of DriveTrain***********************************/
  //Routine being added to all subsystems
  public void Reset(){
    //Set direction to intake is front of robot
    ResetFront();

    //Reset the drivetrain sensors
    resetAngle();
    resetDistance();
    setDriverMode();
  }
  /*Driving Methods**********************************************/
  
  //Resets the Intake to be the Robot's Front
  public void ResetFront(){
    //Make intake the front of the robot
    direction = 1;
    intakeIsFront = true;
  }
  
  //Sets the Drivetrain Motors to speed and rotation of 0.0 to stop all motion
  public void Stop() {
    DriveTrain.arcadeDrive(0, 0);
  }

  public void setBrakeMode(){
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
  }

  //Used as a multiplier to forward speed to reverse the front and back of the robot
  //  Helps the driver by allowing the robot to always be oriented asa if driving forward
  //  insted of one direction being like backing up a car (left and right are reversed)
  public void changeFront() {
    direction *= -1;

    intakeIsFront = !intakeIsFront;
  }

  private double LimitInput(double value){
    //Use Clamp to limit maximum magnitude to +/- DRIVETRAIN_MAXSPEED constant
    //Set to Zero if inside the deadzone for the joystick
    return MathUtil.clamp((Math.abs(value)<Constants.DRIVETRAIN_DEADZONE)?0.0:value,-Constants.DRIVETRAIN_MAXSPEED, Constants.DRIVETRAIN_MAXSPEED);
    //return value;
  }

  //Use Speed and rotation to Drive the Robot, Y axis is forward speed, X axis is rotation
  public void TeleopArcadeDrive(double speed, double rot) {
    //For Testing a new Drivetrain.  Comment out call to arcadeDrive, then enable
    //  each motor, one at a time, by uncommenting the set(speed) command.
    //  Change inversion in the constructors until controller reads green for 
    //  forward control and red for reverse.  Repeat for each motor.
    DriveTrain.arcadeDrive(direction * LimitInput(speed), LimitInput(rot));
    SmartDashboard.putNumber("arcade speed", speed);
    SmartDashboard.putNumber("arcade rot", rot);
    //rightMaster.set(speed);
    //rightSlave.set(speed);
    //leftMaster.set(speed);
    //leftSlave.set(speed);
    
    //Uncomment if we want to see Left and Right Encoder values for testing and tuning
    
    SmartDashboard.putNumber("Right Rate", rightEncoder.getRate());
    SmartDashboard.putNumber("Left Rate ", leftEncoder.getRate());
    SmartDashboard.putNumber("Left Encoder  ", leftEncoder.getRaw());
    SmartDashboard.putNumber("Right Encoder ", rightEncoder.getRaw());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Left Distance ", getLeftDistance());
    SmartDashboard.putNumber("Avg Distance  ", getAverageDistance());
  }
  
  //Used the new 2020 API for driving with Kinematic, PID control for smoother acceleration/deacceleration
  public void TeleopKinematicDrive(final double speed, final double rotation) {
    
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(-speed, 0.0, -rotation));
    double leftOutput  = m_leftPID.calculate (-leftEncoder.getRate(),
                                              -wheelSpeeds.leftMetersPerSecond);
    double rightOutput = m_rightPID.calculate (-rightEncoder.getRate(),
                                               -wheelSpeeds.rightMetersPerSecond);

    //Comment out left and right Master output setting for testing and uncomment
    //  TeleopArcadeDrive so all parameters can be viewed and signs determined
    if (intakeIsFront){
      leftMaster.set (-1*direction * leftOutput);
      rightMaster.set(-1*direction * rightOutput);
  } else {
      leftMaster.set (-1*direction * rightOutput);
      rightMaster.set(-1*direction * leftOutput);
  }
  //TeleopArcadeDrive(speed, rotation);

    SmartDashboard.putNumber("Left Output  ", leftOutput);
    SmartDashboard.putNumber("Right Output ", rightOutput);
    SmartDashboard.putNumber("Right Rate", rightEncoder.getRate());
    SmartDashboard.putNumber("Left Rate ", leftEncoder.getRate());
    SmartDashboard.putNumber("Left Wheel Speed",wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed",wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Speed",speed);
    SmartDashboard.putNumber("Rotation",rotation);
    //Uncomment if we want to see Left and Right Encoder values for testing and tuning
    SmartDashboard.putNumber("Left Encoder  ", leftEncoder.getRaw());
    SmartDashboard.putNumber("Right Encoder ", rightEncoder.getRaw());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Left Distance ", getLeftDistance());
    SmartDashboard.putNumber("Avg Distance  ", getAverageDistance());
  }

  //Resets the accumulated encoder distances to 0.0;
  public void resetDistance(){
    leftEncoder.reset();
    rightEncoder.reset();
  }  

  //Returns the Distance in Meters Recorded by the left side encoder
  public double getLeftDistance(){
    return leftEncoder.getDistance();//*Constants.LEFT_DIST_ADJUSTMENT; //This is being accounted for in Distance Per Pulse now 3/5/20
  }

  //Returns the Distance in Meters Recorded by the right side encoder
  public double getRightDistance(){
    return rightEncoder.getDistance();//*Constants.DIST_ADJUSTMENT; //This is being accounted for in Distance Per Pulse now 3/5/20
  }

  //Returns the Average Distance between the Left and Right side encoders
  public double getAverageDistance(){
    return (getLeftDistance());//+getRightDistance())/2;
  }
  /*End Driving Methods**********************************************/

  /*Limelight Methods************************************************/

  //Places the LimeLight in Normal Camera Mode
  public void setDriverMode()
  {
    limeLight.setCameraMode(true);
    limeLightDriverMode = true;
  }

  //Places the LimeLight in Vision Tracking Mode
  public void setVisionNode()
  {
    limeLight.setCameraMode(false);
    limeLightDriverMode = false;
  }

  //Toggle the current LimeLight Camera Mode between Driver and Vision
  public void toggleLimeLightMode(){
    limeLightDriverMode=!limeLightDriverMode;
    if (limeLightDriverMode) setDriverMode();
    else setVisionNode();
  }

  //Toggle Limelight Taking Snapshots
  public void toggleTakeSnapShots(){
    limeLight.toggleSnapShot();
  }

  //Uses LimeLight Tracking Data to Drive to Target in Arcade Mode
  public boolean visionDriveArcade(){
    if (limeLight.calcDoubleS()){
      //Save the current driving direction and set the Intake to the Front
      //int saveDirection = direction;
      //direction = 1;
      //intakeIsFront = true;
                             //Drive toward target should be negative
      this.TeleopArcadeDrive(limeLight.getDriveCommand(), limeLight.getSteerCommand());
      
      //Reset the front of the Robot to the saved direction
      //direction = saveDirection;
      //intakeIsFront = (direction == 1);
      
      SmartDashboard.putNumber("TX", limeLight.getTX());
      SmartDashboard.putNumber("TY", limeLight.getTY());
      SmartDashboard.putNumber("TA", limeLight.getTA());
      SmartDashboard.putNumber("driveCommand", limeLight.getDriveCommand());
      SmartDashboard.putNumber("steerCommand", limeLight.getSteerCommand());
      return true;
    } else {
      //No Target
      return false;
    }
  }

  //Uses LimeLight Tracking Data to Drive to Target Using Kinematics
  public boolean visionDriveKinematic(){
    if (limeLight.calcDoubleS()){
      this.TeleopKinematicDrive(-limeLight.getDriveCommand()*1.5, 
                                 limeLight.getSteerCommand()*Constants.MAX_ROTATION);
      SmartDashboard.putNumber("TX", limeLight.getTX());
      SmartDashboard.putNumber("TY", limeLight.getTY());
      SmartDashboard.putNumber("TA", limeLight.getTA());
      SmartDashboard.putNumber("driveCommand", limeLight.getDriveCommand());
      SmartDashboard.putNumber("steerCommand", limeLight.getSteerCommand());
      return true;
    } else {
      //No Target
      return false;
    }
  }
  
  //Returns if the LimeLight is Close Enough to the Target
  public boolean atTargetArcade(){
    return limeLight.withinDoubleSTolerance();
  }
  /*End Limelight Methods********************************************/

  /*Pigeon IMU Methods***********************************************/

  //Returns true if the Pigeon IMU is in the Ready State, false otherwise.
  public boolean getIMUStatus(){
    return (pIMU.getState() == PigeonIMU.PigeonState.Ready);
  }

  //Returns the current Gyro Angle normalized to -360 to 360
  //  Pigeon accumlates heading up to a maximum of 64, 360 degree rotations
  //  to normalize to +/-360, the number of rotations is subtracted from the current reading
  public double getAngle(){
    int normalizer=0;
    double angle360=0.0;

    if (getIMUStatus()){
      angle360 = pIMU.getFusedHeading();      //Unormalized Heading +/-23,040 degrees
      normalizer = (int)(angle360/360);       //Cumulative number of 360 revolutions
      angle360 = angle360 - (360*normalizer); // Normalized angle between +/- 360 degrees
    }
    return angle360;
  }

  //Resets the Pigeon IMU's accumulated heading to 0.0
  public void resetAngle(){
    pIMU.setFusedHeading(0.0, Constants.PIGEON_TIMEOUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IMU Heading", getAngle());
  }
}

