/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.intake_motor);
  private boolean on = false;  //Default system to OFF

  /**
  * Creates a new Intake.
  */
  public Intake() {
    intakeMotor.configFactoryDefault();
    //Do we need to set Inverted?
  }

  public void turnOff_ON(){
    on = !on;
    if(on){
      intakeMotor.set(-0.50);
    }
    else{
      intakeMotor.set(0);
    }
  }

  public void unJam(){
    on = true;
    intakeMotor.set(0.60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}