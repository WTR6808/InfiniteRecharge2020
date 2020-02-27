/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeAbility extends CommandBase {
  /**
   * Creates a new IntakeAbility.
   */
  private final Intake m_intake;
  public IntakeAbility(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  //  This will be called every scheduler cycle and will continuously toggle between on/off
  //  Move to initialize so it only runs once
  @Override
  public void execute() {
    m_intake.turnOff_ON();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  //  Should return true so than the command only runs once
  @Override
  public boolean isFinished() {
    return true;
  }
}
