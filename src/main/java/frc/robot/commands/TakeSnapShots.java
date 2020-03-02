/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class TakeSnapShots extends CommandBase {
  private final DriveTrain m_drivetrain;
  //private WaitCommand executionTime;
  /**
   * Creates a new TakeSnapShots.
   */
  public TakeSnapShots(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Take Snapshots for 5 Seconds
    //executionTime = new WaitCommand(5.0);
    m_drivetrain.toggleTakeSnapShots();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turn LimeLight Snapshots Off
    m_drivetrain.toggleTakeSnapShots();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End when time has expired
    return false;//executionTime.isFinished();
  }
}
