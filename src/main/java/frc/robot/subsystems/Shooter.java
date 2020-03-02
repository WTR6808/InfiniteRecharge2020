/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private WPI_VictorSPX shooter_MotorA = new WPI_VictorSPX(Constants.shooter_motorA);
  private WPI_VictorSPX shooter_MotorB = new WPI_VictorSPX(Constants.shooter_motorB);
  private boolean on = false;
  public Shooter() {
    shooter_MotorA.configFactoryDefault();
    shooter_MotorB.configFactoryDefault();
    shooter_MotorB.follow(shooter_MotorA);
    shooter_MotorA.setInverted(true);
    shooter_MotorA.setInverted(InvertType.FollowMaster);

    shooter_MotorA.setNeutralMode(NeutralMode.Brake);
    shooter_MotorB.setNeutralMode(NeutralMode.Brake);
  }
  
  public void turnON_OFF(double pressure){
    shooter_MotorA.set(pressure);
  }

  public void toggle(double power){
    on = !on;
    if (on){
      shooter_MotorA.set(-power);
    }
    else{
      shooter_MotorA.set(0.0);
    }
  }

  public boolean isOff(){
    return !on;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
