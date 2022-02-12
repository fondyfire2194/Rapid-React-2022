// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you ca n modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.GetTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class RunTargetValues extends InstantCommand {
  private GetTarget m_getTgt;

  public RunTargetValues(GetTarget getTgt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_getTgt = getTgt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_getTgt.runCalcs();
  }
}
