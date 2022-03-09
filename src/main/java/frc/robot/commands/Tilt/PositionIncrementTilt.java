// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionIncrementTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private int loopCtr;

  private double m_increment;

  private boolean endIt;


  public PositionIncrementTilt(RevTiltSubsystem tilt, double increment) {
    m_tilt = tilt;
    m_increment = increment;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.programRunning = 2;

    m_tilt.targetAngle += m_increment;

    loopCtr = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;

    m_tilt.goToPositionMotionMagic(m_tilt.targetAngle);

    endIt = m_tilt.atTargetAngle() && loopCtr > 10 && Math.abs(m_tilt.getSpeed()) < 1;// || !m_tilt.positionResetDone;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (loopCtr > 10 && !endIt)
    
      m_tilt.targetAngle = m_tilt.getAngle();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
