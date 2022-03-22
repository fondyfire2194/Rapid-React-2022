// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.LimelightControlMode.LedMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnLedsOnOff extends InstantCommand {
  private LimeLight m_ll;
  private boolean m_on;

  public TurnLedsOnOff(LimeLight ll, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ll = ll;
    m_on = on;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_on)
      m_ll.setLEDMode(LedMode.kpipeLine);
    else
      m_ll.setLEDMode(LedMode.kforceOff);
  }
}
