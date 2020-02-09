/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class VisionDriveToTarget extends CommandBase {
  private final DriveTrain m_driveTrain;
  private boolean noTarget;

  /**
   * Creates a new VisionDriveToTarget.
   */
  public VisionDriveToTarget(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noTarget=false;
    m_driveTrain.setVisionNode();
    m_driveTrain.Stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noTarget = !m_driveTrain.visionDriveArcade(); //returns true if target found
  //  noTarget = !m_driveTrain.visionDriveKinematic(); //returns true if target found
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setDriverMode();
    m_driveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noTarget || m_driveTrain.atTargetArcade();
  }
}
