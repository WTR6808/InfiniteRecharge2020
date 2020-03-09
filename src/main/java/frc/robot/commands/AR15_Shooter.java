/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AR15_Shooter extends SequentialCommandGroup {
  /**
   * Creates a new AR15_Shooter.
   */
  public AR15_Shooter(Hopper hopper,
                      Shooter shooter,
                      DriveTrain drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(sequence(new VisionDriveToTarget(drivetrain).withTimeout(2.5),
                   new FullPowerShooter(1.0 , shooter),
                   new WaitCommand(0.5),
                   new HopperWorks(hopper),
                   new WaitCommand(2.5),
                   new FullPowerShooter(1.0, shooter),
                   new HopperWorks(hopper)
    ));
  }
}
