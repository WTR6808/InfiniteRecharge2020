/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class KinematicDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   *
   */
   private final DriveTrain m_DriveTrain;
   private final DoubleSupplier speed;
   private final DoubleSupplier rotation;
  
  public KinematicDrive(DoubleSupplier Y, DoubleSupplier X, DriveTrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = drivetrain;
    speed = Y;
    rotation = X;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sp=Math.abs(speed.getAsDouble())>0.2?speed.getAsDouble():0.0;
    double rot=Math.abs(rotation.getAsDouble())>0.2?rotation.getAsDouble():0.0;
    //sp=speed.getAsDouble();
    //rot = rotation.getAsDouble();
    m_DriveTrain.TeleopKinematicDrive(sp*Constants.MAX_SPEED, 
                                      rot*Constants.MAX_ROTATION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}