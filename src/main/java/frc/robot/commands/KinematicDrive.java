/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   
   public static double MAX_SPEED(String name, double max_SPEED) {
     double maximum_Speed = SmartDashboard.getNumber("Max Speed", max_SPEED);
  
     SmartDashboard.putNumber("Max Speed", maximum_Speed);
     return maximum_Speed;
   }

   /*public static double MAXSPEED(String key, double MAX_SPEED){

    SmartDashboard.putNumber("Max Speed", MAX_SPEED);
    SmartDashboard.getNumber("Max Speed", MAX_SPEED);
    return SmartDashboard.getNumber("Max Speed", MAX_SPEED);
   }
   */



  
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
    //Handle Joystick Deadzones ~0.2
    double sp =Math.abs(speed.getAsDouble())>Constants.JOYSTICK_MIN?speed.getAsDouble():0.0;
    double rot=Math.abs(rotation.getAsDouble())>Constants.JOYSTICK_MIN?rotation.getAsDouble():0.0;
    
    m_DriveTrain.TeleopKinematicDrive(sp*Constants.MAX_SPEED,//("Max Speed", 3.0), 
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
