// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class RunAtVolts extends CommandBase {
  /** Creates a new RunAtVolts. */
  private RevDrivetrain m_drive;
  private boolean m_minus;
  double maxVolts = 1;
  double minVolts = 0;

  public RunAtVolts(RevDrivetrain drive, boolean minus) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_minus = minus;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.ksVolts = .2;
    if (m_minus)
      m_drive.ksVolts = -.2;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double voltsRange = maxVolts - minVolts;

    m_drive.currentVolts = minVolts + voltsRange * m_drive.voltsValue;

    if (m_minus)
      m_drive.currentVolts = -m_drive.currentVolts;

    m_drive.tankDriveVolts(m_drive.currentVolts, m_drive.currentVolts);

    m_drive.kvVolts = m_drive.currentVolts - m_drive.ksVolts;

    m_drive.leftmpspervolt = m_drive.getLeftRate() / m_drive.kvVolts;

    m_drive.rightmpspervolt = m_drive.getRightRate() / m_drive.kvVolts;

    if (m_minus) {
      m_drive.leftmpspervolt = -m_drive.leftmpspervolt;
      m_drive.rightmpspervolt = -m_drive.rightmpspervolt;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
