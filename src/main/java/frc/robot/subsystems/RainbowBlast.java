/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class RainbowBlast extends SubsystemBase {
  /**
   * Creates a new RainbowBlast.
   */
  private char gameColor;
  private final WPI_VictorSPX centralMotor = new WPI_VictorSPX(Constants.spinner_CentralMotor);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.12, 0.43, 0.45);
  private final Color kRedTarget = ColorMatch.makeColor(0.58, 0.31, 0.09);
  private final Color kGreenTarget = ColorMatch.makeColor(0.15, 0.55, 0.25);
  private final Color kYellowTarget = ColorMatch.makeColor(0.36, 0.52, 0.11);
  //finish recording some of the values

  public RainbowBlast() {
    centralMotor.configFactoryDefault();
    centralMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void SetgameColor(final char color) {
    gameColor = color;

    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }


  }
  
  
 private final void getgameColor() {
   Color detectedColor = colorSensor.getColor();
   SmartDashboard.putNumber("Red", detectedColor.red);
   SmartDashboard.putNumber("Green", detectedColor.green);
   SmartDashboard.putNumber("Blue", detectedColor.blue);
   

 }

}
