/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
  private final Shooter m_shooter;
  private final DoubleSupplier pressure;

  /**
  * Creates a new ShootBall.
  */
  public ShootBall(DoubleSupplier P, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    pressure = P;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_shooter.toggle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Only execute command if shooter has not been toggled on
    if (m_shooter.isOff()){
      m_shooter.turnON_OFF(-pressure.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  //  this is the default command for the Shooter, so make it never end
  @Override
  public boolean isFinished() {
    return false;
  }
}
