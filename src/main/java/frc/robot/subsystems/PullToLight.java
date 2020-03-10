/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class PullToLight extends SubsystemBase {
  /**
   * Creates a new Reaching_For_Gods_Grace.
   */
  private WPI_VictorSPX Hook = new WPI_VictorSPX(Constants.LIFT_ARM);
  private CANSparkMax Winch = new CANSparkMax(Constants.LIFT_MOTOR, MotorType.kBrushless);
  public void GodsHand(){
    Hook.set(0.5);

  }
  public void DevilsGrip(){
    Hook.set(-0.5);

  }
  public void AscensionToHeaven(){
    Winch.set(1.0);

  }
  public PullToLight() {
  Hook.configFactoryDefault();
  Winch.restoreFactoryDefaults();
  Hook.setNeutralMode(NeutralMode.Brake);
  Winch.setIdleMode(IdleMode.kBrake);



  }
  public void Stop(){
    Hook.set(0.0);
  }
  public void Purgatory(){
    Winch.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
