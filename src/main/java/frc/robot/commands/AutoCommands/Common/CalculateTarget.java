// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.GetTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalculateTarget extends InstantCommand {
  private GetTarget m_gt;
  public CalculateTarget(GetTarget gt) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_gt.weightedTargetValue = m_gt.weightedAverageX(); 
    m_gt.weightedTargetAngle = m_gt.getTargetAngle(m_gt.weightedTargetValue);

  }
}
