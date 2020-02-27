/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  /**
   * Creates a new DriveToDistance.
   */
  private final static double DISTANCE_TOLERANCE = 0.05;
  private final DriveTrain m_drivetrain;
  private final double m_speed;
  private final double m_distance;
  private double m_targetdistance;
  private int timeDelay = 0;

  public DriveToDistance(double speed, double distance, DriveTrain driveTrain) {
    m_speed = speed;
    m_distance = distance;
    m_drivetrain = driveTrain;
    addRequirements(m_drivetrain);
    //m_targetdistance = m_drivetrain.getAverageDistance()+distance;

    //SmartDashboard.putNumber("target distance",  m_targetdistance);
    //SmartDashboard.putNumber("distance", m_distance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeDelay = 0;
    m_drivetrain.Stop();
    m_drivetrain.resetDistance();
    m_targetdistance = m_distance;
    SmartDashboard.putNumber("target distance", m_targetdistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Wait 5 cycles to make sure encoders have reset
    //if (++timeDelay>5) m_drivetrain.TeleopArcadeDrive(m_speed, 0.0);
    if (++timeDelay>5) m_drivetrain.TeleopKinematicDrive(Math.signum(m_targetdistance)*(m_speed+0.2), 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.Stop();
    //m_drivetrain.TeleopArcadeDrive(0.0, 0.0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //For Arcade Drive multiply this by -1;
    double avgDist = m_drivetrain.getAverageDistance();
    return (Math.abs(m_targetdistance - avgDist) < DISTANCE_TOLERANCE);
  }
}
