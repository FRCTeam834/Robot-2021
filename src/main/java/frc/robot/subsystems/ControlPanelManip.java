/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
/*
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.commands.PrintColor;
import frc.robot.commands.StopCPManip;
import frc.robot.Constants;

//https://first.wpi.edu/FRC/roborio/development/docs/java/edu/wpi
//https://github.com/REVrobotics/Color-Sensor-v3-Examples/tree/master/Java/Color%20Match

public class ControlPanelManip extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  WPI_VictorSPX cPWheel = new WPI_VictorSPX(Constants.CONTROL_PANEL_MOTOR_PORT);

  public ControlPanelManip() {
    cPWheel.setInverted(Constants.CONTROL_PANEL_INVERTED);

    // add wheel colors to the color match thing so it works
    m_colorMatcher.addColorMatch(Constants.RED_TARGET);
    m_colorMatcher.addColorMatch(Constants.BLUE_TARGET);
    m_colorMatcher.addColorMatch(Constants.GREEN_TARGET);
    m_colorMatcher.addColorMatch(Constants.YELLOW_TARGET);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new StopCPManip());
  }

  public double getRed() {

    colorSensor.getRed();

  }

  public double getBlue() {

    return colorSensor.getBlue();

  }

  // gets green
  public double getGreen() {

    return colorSensor.getGreen();

  }

  // gets raw color
  public Color getRawColor() {

    // read current color and return it
    return colorSensor.getColor();

  }

  // determines wheel color
  public String determineWheelColor() {
    Color detectedColor = getRawColor();

    // determine the color
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // determine % of red used for sensitivity
    double total = getRed() + getGreen() + getBlue();
    int redPercent = (int) (getRed() / total * 1000);
    int greenPercent = (int) (getGreen() / total * 1000);
    int bluePercent = (int) (getBlue() / total * 1000);
    // determine color of the wheel
    if (match.color == Constants.BLUE_TARGET) {
      return "R"; // when sensor reads blue, field reads red
    } else if (match.color == Constants.RED_TARGET) {
      return "B"; // when sensor reads red, field reads blue
    } else if (match.color == Constants.GREEN_TARGET && (redPercent < 200)) {
      return "Y"; // when sensor reads green, field reads yellow
    } else if (match.color == Constants.YELLOW_TARGET && (redPercent < 350) && (greenPercent > 500)) {
      return "G"; // when sensor reads yellow, field reads green
    } else {
      return "NO MATCH";
    }

  }

  public void start() {

    //cPWheel.set(Constants.CP_WHEEL_SPEED);
  
  }

  public void stop() {

   //cPWheel.set(0);

  }

}
*/