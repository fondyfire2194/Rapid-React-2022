// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class TrajSimVolts extends CommandBase {
  /** Creates a new RunAtVolts. */
  private RevDrivetrain m_drive;

  double minVolts = 0;

  double voltsIncrementPer20ms;

  double maxVolts = 2.5;

  double time = 3;

  private double voltsIncremementPerSecond;

  private boolean slowDown;

  private boolean m_minus;

  public TrajSimVolts(RevDrivetrain drive, boolean minus) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_minus = minus;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.currentVolts = 0;
    m_drive.resetEncoders();
    m_drive.resetGyro();
    slowDown = false;
    voltsIncremementPerSecond = maxVolts / (time / 2);

    SmartDashboard.putNumber("VIPS", voltsIncremementPerSecond);

    voltsIncrementPer20ms = voltsIncremementPerSecond / 50;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!slowDown) {

      m_drive.currentVolts += voltsIncrementPer20ms;

      if (m_drive.currentVolts >= maxVolts)
        slowDown = true;
    }

    if (slowDown) {

      m_drive.currentVolts -= voltsIncrementPer20ms;

      if (m_drive.currentVolts < .1)

        m_drive.currentVolts = 0;
    }
    if (m_minus)
      m_drive.tankDriveVolts(-m_drive.currentVolts, -m_drive.currentVolts);
    else
      m_drive.tankDriveVolts(m_drive.currentVolts, m_drive.currentVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.currentVolts = 0;
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
