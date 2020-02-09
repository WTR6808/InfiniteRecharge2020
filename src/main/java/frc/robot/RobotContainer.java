/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.KinematicDrive;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.VisionDriveToTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RainbowBlast;
import frc.robot.commands.ArcadeDrive;

//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final Joystick m_L1 = new Joystick(0);
  private final RainbowBlast centralMotor = new RainbowBlast();


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveTrain.setDefaultCommand(new KinematicDrive(()->m_L1.getY(Hand.kLeft),
                                                     ()->m_L1.getX(Hand.kLeft),
                                                    m_DriveTrain)); 
                                                  
                                                   
    
    // Configure the button bindings
    /*configureButtonBindings();   
     m_DriveTrain.setDefaultCommand(new ArcadeDrive(()->m_L1.getY(Hand.kLeft),
                                                       ()->m_L1.getX(Hand.kLeft),
                                                       m_DriveTrain)); */
  }  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    final JoystickButton driverX_A            = new JoystickButton(m_L1,  1);
    final JoystickButton driverX_B            = new JoystickButton(m_L1,  2);
    final JoystickButton driverX_X            = new JoystickButton(m_L1,  3);
    final JoystickButton driverX_Y            = new JoystickButton(m_L1,  4);
    final JoystickButton driverX_LeftBumper   = new JoystickButton(m_L1,  5);
    final JoystickButton driverX_RightBumper  = new JoystickButton(m_L1,  6);
    final JoystickButton driverX_Back         = new JoystickButton(m_L1,  7);
    final JoystickButton driverX_Start        = new JoystickButton(m_L1,  8);
    final JoystickButton driverX_L3           = new JoystickButton(m_L1,  9);
    final JoystickButton driverX_R3           = new JoystickButton(m_L1, 10);

    driverX_A.whenPressed(new DriveToDistance(-0.5, 2, m_DriveTrain));
    //in meters
    driverX_B.whenPressed(new SpinWheel());
    driverX_X.whenPressed(new VisionDriveToTarget(m_DriveTrain));


}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
