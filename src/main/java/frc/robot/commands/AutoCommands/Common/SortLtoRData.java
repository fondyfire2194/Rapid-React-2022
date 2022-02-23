// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Vision.RawContoursV2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SortLtoRData extends InstantCommand {
  private RawContoursV2 m_rcv2;
private double startTime;
  public SortLtoRData(RawContoursV2 rcv2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rcv2 = rcv2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_rcv2.startTime=m_rcv2.getStartTime();

    
    m_rcv2.getHubVisionData();

    SmartDashboard.putNumber("SortCmdTime", m_rcv2.getEndTime(m_rcv2.startTime));

  }
}
