/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hopper;

public class ShortHop extends CommandBase {
  /**
   * Creates a new ShortHop.
   */
  private final Hopper m_hopper;
  //private WaitCommand executionTime;


  public ShortHop(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //executionTime = new WaitCommand(.2);
    m_hopper.turnON_OFF();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.turnON_OFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//executionTime.isFinished();
  }
}
