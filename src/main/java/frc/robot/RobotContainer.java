/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.HopperWorks;
import frc.robot.commands.IntakeAbility;
import frc.robot.commands.KinematicDrive;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.TurningAngle;
import frc.robot.commands.VisionDriveToTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RainbowBlast;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.Autonomous3;
import frc.robot.commands.AutonomousShootBalls;
import frc.robot.commands.Darkness;

//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Define the Joysticks
  private final XboxController m_Driver = new XboxController(0);
  private final XboxController m_Helper = new XboxController(1);

   // The robot's subsystems and commands are defined here...
  private final DriveTrain m_DriveTrain = new DriveTrain();
  //private final RainbowBlast centralMotor = new RainbowBlast();
  private final Hopper hopperMotor = new Hopper();
  private final Shooter shooterMotor = new Shooter();
  private final Intake intakeMotor = new Intake();

  private SendableChooser <Command> m_Chooser = new SendableChooser<Command>();
  //private Command m_autoCommand = new Autonomous1(m_DriveTrain);//new ExampleCommand(m_exampleSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Setup the Default Commands
    //DriveTrain using new Kinematics for driving
    m_DriveTrain.setDefaultCommand(new KinematicDrive(()->m_Driver.getY(Hand.kLeft),
                                                      ()->m_Driver.getX(Hand.kLeft),
                                                      m_DriveTrain)); 
    
    //DriveTrain using older style ArcadeDrive for driving
    //m_DriveTrain.setDefaultCommand(new ArcadeDrive(()->m_L1.getY(Hand.kLeft),
    //                                               ()->m_L1.getX(Hand.kLeft),
    //                                               m_DriveTrain)); 

    //Trigger control of Ball Shooter Motor
    //If we are using the Left Joystick, should we use the Left Trigger, or Right Trigger?
    //  Ask drivers.  Also, should shooter be variable speed?                                                      
    //shooterMotor.setDefaultCommand(new ShootBall(()-> m_L1.getTriggerAxis(Hand.kLeft), shooterMotor));
    
    // Configure the button bindings
    configureButtonBindings();   

    //Create our Sendable Chooser Here
//    m_Chooser.addOption("No Auto", null);
    m_Chooser.setDefaultOption("No Auto", null);
    m_Chooser.addOption("Position Left", new Autonomous2(m_DriveTrain,shooterMotor, hopperMotor));
    m_Chooser.addOption("Position Middle", new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor));
    m_Chooser.addOption("Position Right", new Autonomous3(m_DriveTrain, shooterMotor, hopperMotor));
    SmartDashboard.putData("Autonomous Selection", m_Chooser);

    //Create the USB Camera Stream for Microsoft LifeCam
    CameraServer.getInstance().startAutomaticCapture();

    //Add usefull debugging information to our dashboard
    //  Comment out for competition?
    //Display the currently executing command
    SmartDashboard.putData(CommandScheduler.getInstance());
    //Display the status of the subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(shooterMotor);
    SmartDashboard.putData(hopperMotor);
    SmartDashboard.putData(intakeMotor);
  }  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    //Driver Controller Button Definitions
    final JoystickButton driverX_A            = new JoystickButton(m_Driver,  1);
    final JoystickButton driverX_B            = new JoystickButton(m_Driver,  2);
    final JoystickButton driverX_X            = new JoystickButton(m_Driver,  3);
    final JoystickButton driverX_Y            = new JoystickButton(m_Driver,  4);
    final JoystickButton driverX_LeftBumper   = new JoystickButton(m_Driver,  5);
    final JoystickButton driverX_RightBumper  = new JoystickButton(m_Driver,  6);
    final JoystickButton driverX_Back         = new JoystickButton(m_Driver,  7);
    final JoystickButton driverX_Start        = new JoystickButton(m_Driver,  8);
    final JoystickButton driverX_L3           = new JoystickButton(m_Driver,  9);
    final JoystickButton driverX_R3           = new JoystickButton(m_Driver, 10);

    //Driver's Helper Button Definitiions
    final JoystickButton Helper_A            = new JoystickButton(m_Helper,  1);
    final JoystickButton Helper_B            = new JoystickButton(m_Helper,  2);
    final JoystickButton Helper_X            = new JoystickButton(m_Helper,  3);
    final JoystickButton Helper_Y            = new JoystickButton(m_Helper,  4);
    final JoystickButton Helper_LeftBumper   = new JoystickButton(m_Helper,  5);
    final JoystickButton Helper_RightBumper  = new JoystickButton(m_Helper,  6);
    final JoystickButton Helper_Back         = new JoystickButton(m_Helper,  7);
    final JoystickButton Helper_Start        = new JoystickButton(m_Helper,  8);
    final JoystickButton Helper_L3           = new JoystickButton(m_Helper,  9);
    final JoystickButton Helper_R3           = new JoystickButton(m_Helper, 10);


    //Mappings for Competition
    driverX_A.whenPressed(new ToggleShooter(shooterMotor));
    driverX_B.whenPressed(new HopperWorks(hopperMotor));
    driverX_L3.whenPressed(new IntakeAbility(intakeMotor));
    driverX_X.whenPressed(new VisionDriveToTarget(m_DriveTrain));
   
    //Should we change this to a toggle?
    driverX_Y.whenPressed(new Darkness(m_DriveTrain));


    //Mappings for Testing/Tuning robot.  Comment out for competition
    driverX_RightBumper.whenPressed(new TurningAngle(90.0, m_DriveTrain));
    driverX_LeftBumper.whenPressed(new DriveToDistance(0.7, 2, m_DriveTrain));
    //in meters
    //driverX_R3.whenPressed(new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor));
    driverX_R3.whenPressed(new AutonomousShootBalls(shooterMotor,hopperMotor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return null;
    return m_Chooser.getSelected();
  }
}
