// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

public final class Show_Hide_Screens {

  private Show_Hide_Screens() {
  }

  public static void setStates(boolean auto, boolean vision, boolean test) {

    boolean autoMode = auto;

    boolean showVision = vision;

    boolean testing = test;

    SetUpOI.showTurret = testing;
    SetUpOI.showTilt = testing;
    SetUpOI.showShooter = testing;
    SetUpOI.showRobot = testing;
    SetUpOI.showTransport = testing;
    SetUpOI.showClimber = testing;
    SetUpOI.showSubsystems = testing || auto;
    SetUpOI.showIntake = testing;

    LLVisionShuffleboard.m_showVision = showVision;

    SetUpPreRoundOI.m_showPreRound = autoMode;

    SetUpAutoOI.m_showAuto = autoMode;

  }
}
