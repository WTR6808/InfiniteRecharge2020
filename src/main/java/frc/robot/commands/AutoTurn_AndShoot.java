/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTurn_AndShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoTurn_AndShoot.
   */
  DriveTrain m_driveTrain;
  Hopper m_hopper;
  Shooter m_shooter;
  public AutoTurn_AndShoot(Hopper hopper, Shooter shooter, DriveTrain drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(sequence(new AR15_Shooter(hopper, shooter, drivetrain),
                   new DriveToDistance(0.65, -1.0, drivetrain)
          ));

  }
}
