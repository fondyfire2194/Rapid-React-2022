// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private int loopCtr;

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
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawKp = 0;
    loopCtr++;

    if (RobotBase.isReal())

      yawKp = Pref.getPref("dRStKp");

    else {

      yawKp = .0;

      m_drive.driveDistance(m_endpoint);

      m_drive.driveDistance(m_endpoint);
    }

    double yawError = 0;

    double yawCorrection = 0;

    yawError = m_drive.getYaw() - m_startAngle;

    yawCorrection = yawError * yawKp;

    leftOut = m_endpoint - m_drive.getLeftDistance();

    rightOut = m_endpoint - m_drive.getRightDistance();

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

    SmartDashboard.putNumber("Lout", leftOut);
    SmartDashboard.putNumber("Rout", rightOut);
    SmartDashboard.putNumber("LPos", m_drive.getLeftDistance());
    SmartDashboard.putNumber("RPos", m_drive.getRightDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftOut = 0;
    rightOut = 0;
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return loopCtr > 5 && Math.abs(m_endpoint - m_drive.getLeftDistance()) < .1

        && Math.abs(m_endpoint - m_drive.getRightDistance()) < .1;

    // && m_drive.isStopped();
  }
}
