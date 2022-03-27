// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class RevColorSensor {
  /** Creates a new RevColorSensor. */

  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * detectedColor = m_colorSensor.getColor();
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.55, 0.22, 0.11);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  Color detectedColor;
  ColorMatchResult match;
  public ColorSensorV3 m_colorSensor;

  public RevColorSensor(ColorSensorV3 colorSensor) {
    m_colorSensor = colorSensor;

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

  }

  public ColorMatchResult getDetectedColor() {

    detectedColor = m_colorSensor.getColor();

    match = m_colorMatcher.matchClosestColor(detectedColor);

    return match;
  }

  public boolean getIsBlue() {

    return getDetectedColor().color == kBlueTarget;

  }

  public boolean getIsRed() {

    return getDetectedColor().color == kRedTarget;
  }

  public String getColorString() {
    String colorString;
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

    SmartDashboard.putString("Color", colorString);
    return colorString;
  }

  public int getProximity() {
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("ColorProx", proximity);

    return proximity;

  }

  public double getIR() {
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("ColorIR", IR);

    return IR;
  }
}
