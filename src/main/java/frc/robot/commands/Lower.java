/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PullToLight;

public class Lower extends CommandBase {
  /**
   * Creates a new Lower.
   */
  PullToLight pullToLight;
  public Lower(PullToLight pulltolight) {
    // Use addRequirements() here to declare subsystem dependencies.
    pullToLight = pulltolight;
    addRequirements(pullToLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pullToLight.DevilsGrip();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pullToLight.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
