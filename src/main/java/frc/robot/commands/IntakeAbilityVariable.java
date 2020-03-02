/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeAbilityVariable extends CommandBase {
  private final Intake m_intake;
  private final DoubleSupplier m_power;
  /**
   * Creates a new IntakeAbilityVariable.
   */
  public IntakeAbilityVariable(DoubleSupplier power, Intake intake) {
    m_intake = intake;
    m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Only execute command if intake has not been toggled on
    //if (m_intake.isOff()){
      m_intake.setSpeed(-m_power.getAsDouble()/1.5); //Divide by two to limit from 0 to 0.5
    //}
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
