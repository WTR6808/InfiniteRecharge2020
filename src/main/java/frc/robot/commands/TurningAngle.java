/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurningAngle extends CommandBase {
  /**
   * Creates a new TurningAngle.
   */
  private double targetAngle =0.0;
  
  //Need to move these to Constants and Adjust - Speed is currently to fast and 10 degree accurracy is awful
  //private final static double HEADING_TOLERANCE = 10.0;
  private double speedAdj = Constants.DRIVETRAIN_MAX_ROT;

  private final DriveTrain m_drivetrain;

  //We need to delay to allow time for the reset angle routine to take affect
  private int delay = 0;

  public TurningAngle(double target, DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    targetAngle = target;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.Stop();
    m_drivetrain.resetAngle();
    delay = 0;
    SmartDashboard.putNumber("Target Angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Add delay to allow for angle to reset before starting to execute movement
    if (delay > 10){
      //speedAdj should be a constant so it does not change in multiple places
      //speedAdj = 5;
      //m_drivetrain.TeleopKinematicDrive(0.0, -speedAdj*Math.signum(targetAngle));
      m_drivetrain.TeleopArcadeDrive(0.0, speedAdj * Math.signum(targetAngle));
//      SmartDashboard.putNumber("Target Angle", targetAngle);
//      speedAdj= SmartDashboard.getNumber("Speed Adj", 0.0);
//      SmartDashboard.putNumber("New Adj", speedAdj);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //We should stop the drivetrain here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (delay++ >= 10) && (Math.abs(targetAngle) - Math.abs(m_drivetrain.getAngle()) < Constants.HEADING_TOLERANCE);
  }
}