/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  //Motor B is Master, Motor A is Slave
  private WPI_VictorSPX hopperMotorA = new WPI_VictorSPX(Constants.hopper_motorA);
  private WPI_VictorSPX hopperMotorB = new WPI_VictorSPX(Constants.hopper_motorB);
  private boolean on = false;  //Initialize system to Off

  /**
  * Creates a new Hopper.
  */
  public Hopper() {
    hopperMotorA.configFactoryDefault();
    hopperMotorB.configFactoryDefault();
    hopperMotorA.follow(hopperMotorB);

    //Need to verify setting of Inverted
    hopperMotorB.setInverted(true);
    hopperMotorA.setInverted(InvertType.FollowMaster);
  }

  //Toggles the state of the belt drive between on/off
  //  Will we need to control speed from a trigger?
  public void turnON_OFF(){
    on = !on;
    if(on) hopperMotorB.set(-0.90);
    else hopperMotorB.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}