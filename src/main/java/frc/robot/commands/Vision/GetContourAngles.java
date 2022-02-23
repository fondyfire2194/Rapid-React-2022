// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.AngleSolver;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.VisionReferenceTarget;

public class GetContourAngles extends CommandBase {
  /** Creates a new GetContouurAreas. */
  private AngleSolver m_as;;
  private RawContoursV2 m_rcv2;
  private VisionReferenceTarget m_vrt;
  private boolean areasOK;
  private double m_startTime;

  public GetContourAngles(AngleSolver as,RawContoursV2 rcv2,VisionReferenceTarget vrt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_as = as;
    m_rcv2 =rcv2;
    m_vrt=vrt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_as.lrTxAngles = m_as.getTxVpAngles(m_rcv2.lTRIndex);

    m_as.lrTyAngles = m_as.getTyVpAngles(m_rcv2.lTRIndex);

    m_vrt.testAngles =m_vrt.getTestTargetVPAngles();

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
