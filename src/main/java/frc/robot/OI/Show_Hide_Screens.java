// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import frc.robot.trajectories.FondyFireTrajectory;

public final class Show_Hide_Screens {

  private Show_Hide_Screens() {
  }

  public static void setStates(boolean auto, boolean robotShooter, boolean tiltTurret,
      boolean trajectories, boolean transportIntakeClimber, boolean vision) {

    SetUpAutoOI.m_showAuto = auto;
    SetUpPreRoundOI.m_showPreRound = auto;

    SetUpOI.showSubsystems = true;

    SetUpOI.showRobot = robotShooter;
    SetUpOI.showShooter = robotShooter;

    SetUpOI.showTilt = tiltTurret;
    SetUpOI.showTurret = tiltTurret;

    FondyFireTrajectory.showTrajectories = trajectories;

    SetUpOI.showTransport = transportIntakeClimber;
    SetUpOI.showIntake = transportIntakeClimber;
    SetUpOI.showClimber = transportIntakeClimber;

    LLVisionShuffleboard.m_showVision = vision;

  }
}
