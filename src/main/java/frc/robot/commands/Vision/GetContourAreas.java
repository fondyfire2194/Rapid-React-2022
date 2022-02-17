// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.RawContoursV2;

public class GetContourAreas extends CommandBase {
  /** Creates a new GetContouurAreas. */
  private RawContoursV2 m_rcv2;
  private boolean areasOK;
  private double m_startTime;

  public GetContourAreas(RawContoursV2 rcvv2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rcv2 = rcvv2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    areasOK = m_rcv2.getAreaData();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return areasOK || Timer.getFPGATimestamp() > m_startTime + 1.0;
  }
}
