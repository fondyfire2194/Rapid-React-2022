// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.RawContoursV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalculateTarget extends InstantCommand {
  private RawContoursV2 m_rcv2;
  

  public CalculateTarget(RawContoursV2 rcv2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rcv2 = rcv2;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_rcv2.weightedTargetValue = m_rcv2.weightedAverageX();

    m_rcv2.weightedTargetAngle = m_rcv2.getTargetAngle(m_rcv2.weightedTargetValue);

  }
}
