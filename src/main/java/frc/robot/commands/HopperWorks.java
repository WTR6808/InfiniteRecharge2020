/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class HopperWorks extends CommandBase {
  /**
   * Creates a new HopperWorks.
   */
  private final Hopper m_Hopper;
  
  public HopperWorks(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Hopper = hopper;
    
    addRequirements(m_Hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Hopper.turnON_OFF();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Placing turnONOFF() call here will cause the hopper to toggle between on and off
  //   every other cycle.  Move this call to Initialize so it is only called once.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // Command will never end, this should only be used for a default command.
  //   Since this command is a toggle, it only needs to run once and then complete
  //   return true instead.
  @Override
  public boolean isFinished() {
    return true;
  }
}
