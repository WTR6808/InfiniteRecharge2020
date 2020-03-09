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
import frc.robot.commands.FullPowerShooter;
import frc.robot.commands.HopperWorksVariable;
import frc.robot.commands.IntakeAbilityVariable;
import frc.robot.commands.ReverseHopper;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.TakeSnapShots;
import frc.robot.commands.TurningAngle;
import frc.robot.commands.VisionDriveToTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoTurn_AndShoot;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.Autonomous3;
import frc.robot.commands.ChangeFront;
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
  //private final XboxController m_Helper = new XboxController(1);

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
    //m_DriveTrain.setDefaultCommand(new KinematicDrive(()->m_Driver.getY(Hand.kLeft),
    //                                                  ()->m_Driver.getX(Hand.kLeft),
    //                                                  m_DriveTrain)); 
    
    //DriveTrain using older style ArcadeDrive for driving
    m_DriveTrain.setDefaultCommand(new ArcadeDrive(()->m_Driver.getY(Hand.kLeft),
                                                   ()->m_Driver.getX(Hand.kLeft),
                                                   m_DriveTrain)); 

    //Trigger control of Ball Intake Motor
    //We are using the Left Joystick for driving, use the Right Trigger for the intake                                                      
    intakeMotor.setDefaultCommand(new IntakeAbilityVariable(()-> m_Driver.getTriggerAxis(Hand.kRight), intakeMotor));
    hopperMotor.setDefaultCommand(new HopperWorksVariable(() -> m_Driver.getTriggerAxis(Hand.kLeft), hopperMotor));
    
    // Configure the button bindings
    configureButtonBindings();   

    //Create the USB Camera Stream for Microsoft LifeCam
    CameraServer.getInstance().startAutomaticCapture();

    //Create our Sendable Chooser Here
    //    m_Chooser.addOption("No Auto", null);
    m_Chooser.addOption("No Auto", null);
    m_Chooser.addOption("Position Left", new Autonomous2(m_DriveTrain,shooterMotor, hopperMotor));
    m_Chooser.addOption("Position Middle", new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor));
    m_Chooser.addOption("Position Right", new Autonomous3(m_DriveTrain, shooterMotor, hopperMotor));
    m_Chooser.setDefaultOption("Inline with Target", new AutoTurn_AndShoot(hopperMotor, shooterMotor, m_DriveTrain));
    SmartDashboard.putData("Autonomous Selection", m_Chooser);

    SmartDashboard.putNumber("Selected Command", 4);
  }  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    //Driver Controller Button Definitions
    final JoystickButton driver_A            = new JoystickButton(m_Driver,  1);
    final JoystickButton driver_B            = new JoystickButton(m_Driver,  2);
    final JoystickButton driver_X            = new JoystickButton(m_Driver,  3);
    final JoystickButton driver_Y            = new JoystickButton(m_Driver,  4);
    final JoystickButton driver_LeftBumper   = new JoystickButton(m_Driver,  5);
    final JoystickButton driver_RightBumper  = new JoystickButton(m_Driver,  6);
    final JoystickButton driver_Back         = new JoystickButton(m_Driver,  7);
    final JoystickButton driver_Start        = new JoystickButton(m_Driver,  8);
    final JoystickButton driver_L3           = new JoystickButton(m_Driver,  9);
    final JoystickButton driver_R3           = new JoystickButton(m_Driver, 10);

    //Driver's Helper Button Definitiions
  /*
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
  */

    //Mappings for Competition
    driver_Y.whenPressed(new FullPowerShooter(1.0,shooterMotor));                 //Toggle Shooter Wheels On/Off
    driver_X.whenPressed(new ChangeFront(m_DriveTrain));                          //Reverses the Front of the Robot (Intake is normally the Front)
    driver_A.whenPressed(new Darkness(m_DriveTrain));                             //Toggle the Limelight between Driver and Vision Tracking modes
    driver_B.whenPressed(new TakeSnapShots(m_DriveTrain).withTimeout(2));         //Cause Limelight to Take Snapshots of the Target for 2 seconds

    //Right Trigger default Command for Intake                                    //Right Trigger is Forward Control of Intake Motor
    driver_RightBumper.whileHeld(new ReverseIntake(intakeMotor));                 //Reverse the Intake Motor to UnJam Balls
    //Left Trigger default Command for Hopper                                     //Left Trigger is Forward Control of Hopper Motor
    driver_LeftBumper.whileHeld(new ReverseHopper(hopperMotor));                  //Reverse the Hopper until Left Bumper button is released

    driver_Start.whenPressed(new VisionDriveToTarget(m_DriveTrain));              //Line up the Robot with the target using the Limelight.  Timeout after 2 seconds
    //driver_Back.whenPressed(new ReverseHopper(hopperMotor).withTimeout(.2));    //Currently Unused

    driver_L3.whenPressed(new DriveToDistance(0.65, 1.0, m_DriveTrain));          //Drive Forward at 65% speed for 1 meter.  For Testing, Remove for Competition.
    driver_R3.whenPressed(new TurningAngle(90.0, m_DriveTrain));                  //Turn 90 degrees clockwise.  For Testing, Remove for Competition.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  
  public Command getAutonomousCommand() {
    //return new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor);
    double selection=4.0;
    // An ExampleCommand will run in autonomous
    //Left  Position
    //return new Autonomous2(m_DriveTrain, shooterMotor, hopperMotor);
     
    //Middle Position
    //return new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor);

    //Right Position
    //return new Autonomous3(m_DriveTrain, shooterMotor, hopperMotor);
    
    //Lineup/Shoot and Move off line
    //return new AutoTurn_AndShoot(hopperMotor, shooterMotor, m_DriveTrain);
    //return m_Chooser.getSelected();

    selection = SmartDashboard.getNumber("Selected Command",4.0);
    

    if(selection == 1.0) {
      //Lined up at Far Left
      return new Autonomous2(m_DriveTrain, shooterMotor, hopperMotor);
    }
    else if(selection == 2.0){
      //Lined up in Center of Field
      return new Autonomous1(m_DriveTrain, shooterMotor, hopperMotor);
    }
    else if(selection == 3.0){
      //Lined up at Far Right
      return new Autonomous3(m_DriveTrain, shooterMotor, hopperMotor);
    }
    else if(selection ==4.0){
      //Lined up near in line with target (Preferred and default position)
      return new AutoTurn_AndShoot(hopperMotor, shooterMotor, m_DriveTrain);
    }
    else{
      //Fail Safe in case selection is read incorrectly will guarentee getting of the line
      return new DriveToDistance(0.7, 1.25, m_DriveTrain);  
    }
}

  public void AutonomousInit(){
    m_DriveTrain.setBrakeMode();
  }

  public void TeleopInit(){
    m_DriveTrain.Reset();
    m_DriveTrain.setCoastMode();
  }

  public void RobotPeriodic(){
    //Add usefull debugging information to our dashboard
    //  Comment out for competition
    //Display the currently executing command
    SmartDashboard.putData(CommandScheduler.getInstance());
    //Display the status of the subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(shooterMotor);
    SmartDashboard.putData(hopperMotor);
    SmartDashboard.putData(intakeMotor);
  }
}