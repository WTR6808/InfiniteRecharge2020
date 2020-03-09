/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  // private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.intake_motor);
  private CANSparkMax intakeMotor = new CANSparkMax (Constants.intake_motor, MotorType.kBrushless); 
  private boolean on = false;  //Default system to OFF

  /**
  * Creates a new Intake.
  */
  public Intake() {
    //intakeMotor.configFactoryDefault(); (victor spx motor)
    //Do we need to set Inverted?
    //intakeMotor.setNeutralMode(NeutralMode.Brake); (victor spx motor)
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);

  }

  public void turnOff_ON(){
    on = !on;
    if(on){
      intakeMotor.set(Constants.INTAKE_SPEED);
    }
    else{
      intakeMotor.set(0);
    }
  }

  public void unJam(){
    on = true;
    intakeMotor.set(Constants.intake_reverse);//-1.0);//Constants.intake_reverse);
  }

  public void setSpeed(double speed){
    intakeMotor.set(-speed); //- infront of speed to reverse 
  }

  public boolean isOff(){
    return !on;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}