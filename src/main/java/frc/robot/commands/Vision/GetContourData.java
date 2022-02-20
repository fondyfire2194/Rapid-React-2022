// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.RawContoursV2;

public class GetContourData extends CommandBase {
  /** Creates a new GetContourData. */
  private RawContoursV2 m_rcv2;
  private double m_startTime;

  public GetContourData(RawContoursV2 rcv2) {
    m_rcv2 = rcv2;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    m_startTime = Timer.getFPGATimestamp();

   m_startTime = Timer.getFPGATimestamp();
    m_rcv2.larea.reset();
    m_rcv2.carea.reset();
    m_rcv2.rarea.reset();
    
    m_rcv2.ltx.reset();
    m_rcv2.ctx.reset();
    m_rcv2.rtx.reset();



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_rcv2.getAreaData();
    m_rcv2.getlrtxData();
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("ATXtime", Timer.getFPGATimestamp() - m_startTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rcv2.isDone();
  }
}
