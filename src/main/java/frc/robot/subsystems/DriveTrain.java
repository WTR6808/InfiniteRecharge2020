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
import frc.robot.Constants;
import frc.robot.classes.LimeLight;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private WPI_VictorSPX leftMaster = new WPI_VictorSPX(Constants.drive_LeftFrontMotor);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.drive_LeftRearMotor);
  private WPI_VictorSPX rightMaster = new WPI_VictorSPX(Constants.drive_RightFrontMotor);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.drive_RightRearMotor);
  private DifferentialDrive DriveTrain = new DifferentialDrive(leftMaster, rightMaster);

  private Encoder leftEncoder = new Encoder(Constants.encoder_LeftA, Constants.encoder_LeftB, false,
      CounterBase.EncodingType.k2X);
  private Encoder rightEncoder = new Encoder(Constants.encoder_RightA, Constants.encoder_RightB, true,
      CounterBase.EncodingType.k2X);
  private int direction = 1;

  private final PIDController m_leftPID = new PIDController(.35, 0, 0);
  private final PIDController m_rightPID = new PIDController(.35, 0, 0);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.MTRACK_WIDTH);

  //Create a LimeLight Camera object
  private final LimeLight limeLight = new LimeLight();
  
  public DriveTrain() {
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // Winged Hussars May need Work
    rightMaster.setInverted(false);
    leftMaster.setInverted(true);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    DriveTrain.setRightSideInverted(false);

    // Program the Encoders
    leftEncoder.setDistancePerPulse(Constants.MDIST_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.MDIST_PER_PULSE);
    resetDistance();
    
    //Initialize the LimeLight to Driver Mode
    setDriverMode();
  }

  public void Stop() {
    DriveTrain.arcadeDrive(0, 0);
  }

  public void changeFront() {
    direction *= -1;
  }

  public void TeleopArcadeDrive(double speed, double rot) {
    DriveTrain.arcadeDrive(direction * speed, -rot);
    SmartDashboard.putNumber("Left Encoder  ", leftEncoder.getRaw());
    SmartDashboard.putNumber("Right Encoder ", rightEncoder.getRaw());
    SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Distance ", leftEncoder.getDistance());
  }

  public void TeleopKinematicDrive(final double speed, final double rotation) {
    
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(-speed, 0.0, -rotation));
    double leftOutput = m_leftPID.calculate (-leftEncoder.getRate(),
                                             -wheelSpeeds.leftMetersPerSecond);
    double rightOutput = m_rightPID.calculate (-rightEncoder.getRate(),
                                               -wheelSpeeds.rightMetersPerSecond);
    leftMaster.set(leftOutput);
    rightMaster.set(rightOutput);
    SmartDashboard.putNumber("Left Output  ", leftOutput);
    SmartDashboard.putNumber("Right Output ", rightOutput);
    SmartDashboard.putNumber("Right Rate", rightEncoder.getRate());
    SmartDashboard.putNumber("Left Rate ", leftEncoder.getRate());
    SmartDashboard.putNumber("Left Wheel Speed",wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed",wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Speed",speed);
    SmartDashboard.putNumber("Rotation",rotation);
    //TeleopArcadeDrive(speed, rotation);
   
  }

  public void resetDistance(){
    leftEncoder.reset();
    rightEncoder.reset();
  }  

  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return rightEncoder.getDistance();
  }
  public double getAverageDistance(){
    return (getLeftDistance()+getRightDistance())/2;

  }

  //Limelight Methods
  //Places the LimeLight in Normal Camera Mode
  public void setDriverMode()
  {
    limeLight.setCameraMode(true);
  }

  //Places the LimeLight in Vision Tracking Mode
  public void setVisionNode()
  {
    limeLight.setCameraMode(false);
  }

  //Uses LimeLight Tracking Data to Drive to Target in Arcade Mode
    public boolean visionDriveArcade(){
    if (limeLight.calcDoubleS()){
      this.TeleopArcadeDrive(-limeLight.getDriveCommand(), limeLight.getSteerCommand());
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
      this.TeleopKinematicDrive(-limeLight.getDriveCommand(), limeLight.getSteerCommand());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

