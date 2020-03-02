/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Cancel extends CommandBase {
  /**
   * Creates a new Cancel.
   */
  Shooter m_shooter;
  Hopper m_hopper;
  Intake m_intake;
  public Cancel(Shooter shooter, Hopper hopper, Intake intake) {
    m_shooter = shooter;
    m_hopper = hopper;
    m_intake = intake;
    addRequirements(m_shooter);
    addRequirements(m_hopper);
    addRequirements(m_intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.setSpeed(0.0); 
    m_intake.setSpeed(0.0);
    m_shooter.toggle(0.0);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
