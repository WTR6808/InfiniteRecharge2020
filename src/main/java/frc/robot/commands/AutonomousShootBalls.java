/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousShootBalls extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousShootBalls.
   */
  public AutonomousShootBalls(Shooter shooter, Hopper hopper) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ToggleShooter(shooter),
      new WaitCommand(1.0), //Need to tweek.  Allow Shooter to spin up to fullspeed
      new HopperWorks(hopper),
      new WaitCommand(2.0), //Need to tweek
      new HopperWorks(hopper),
      new ToggleShooter(shooter)
    );
  }
}
