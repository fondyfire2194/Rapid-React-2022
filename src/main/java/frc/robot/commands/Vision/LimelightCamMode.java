// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;

public class LimelightCamMode extends CommandBase {
  /** Creates a new LimelightLeds. */
  private final LimeLight m_limelight;
  private final CamMode m_mode;
  private double m_startTime;

  public LimelightCamMode(LimeLight limelight, CamMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_mode = mode;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setCamMode(m_mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + .1;
  }
}
