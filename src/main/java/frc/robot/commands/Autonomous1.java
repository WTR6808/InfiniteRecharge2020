/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous1 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous1.
   */
  DriveTrain m_driveTrain;
  
  public Autonomous1(DriveTrain driveTrain, Shooter shooter, Hopper hopper)
  {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      sequence( new DriveToDistance(2.0, 1.25, driveTrain), //Speed was 0.8
                new TurningAngle(90, driveTrain),
                new DriveToDistance(2.0, Units.inchesToMeters(77), driveTrain),//-77
                new TurningAngle(90, driveTrain),
                new AR15_Shooter(hopper, shooter, driveTrain)
    ));
  }
}

