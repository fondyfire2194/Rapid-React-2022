// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;

public class PositionStraight extends CommandBase {
  /** Creates a new PositionStraight. */
  private RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_max;
  private double m_min;
  private double m_startAngle;
  private double leftOut;
  private double rightOut;

  public PositionStraight(RevDrivetrain drive, double endPoint, double max) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_endpoint = endPoint;
    m_max = max;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startAngle = m_drive.getYaw();
    m_min = -m_max;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leftOut = m_drive.driveDistance(m_endpoint)[0];

    rightOut = m_drive.driveDistance(m_endpoint)[1];

    double yawError = m_drive.getYaw() - m_startAngle;

    double yawCorrection = yawError * Pref.getPref("dRStKp");

    if (leftOut > m_max)
      leftOut = m_max;
    if (leftOut < m_min)
      leftOut = m_min;

    if (rightOut > m_max)
      rightOut = m_max;
    if (rightOut < m_min)
      rightOut = m_min;

    m_drive.driveLeftSide(leftOut - .5 * yawCorrection);

    m_drive.driveRightSide(rightOut + .5 * yawCorrection);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_endpoint - m_drive.getLeftDistance()) < .1;
  }
}
