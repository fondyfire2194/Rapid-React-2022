// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class TrajSimMPS extends CommandBase {
  /** Creates a new RunAtMPS. */
  private RevDrivetrain m_drive;

  double minMPS = 0;

  double mpsIncrementPer20ms;

  double maxMPS = 2.5;

  double time = 3;

  private double mpsIncremementPerSecond;

  private boolean slowDown;

  private boolean m_minus;

  public TrajSimMPS(RevDrivetrain drive, boolean minus) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_minus = minus;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.currentMPS = 0;
    m_drive.resetEncoders();
    m_drive.resetGyro();
    slowDown = false;
    mpsIncremementPerSecond = maxMPS / (time / 2);

  

    mpsIncrementPer20ms = mpsIncremementPerSecond / 50;
  SmartDashboard.putNumber("VIPS", mpsIncrementPer20ms);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
    SmartDashboard.putBoolean("SLDN", slowDown);

    if (!slowDown) {

      m_drive.currentMPS += mpsIncrementPer20ms;
  SmartDashboard.putNumber("CMPS", m_drive.currentMPS);
      if (m_drive.currentMPS >= maxMPS)
        slowDown = true;
    }

    if (slowDown) {

      m_drive.currentMPS -= mpsIncrementPer20ms;

      if (m_drive.currentMPS < .1)

        m_drive.currentMPS = 0;
    }
    if (m_minus)

      m_drive.tankDriveWithFeedforward(-m_drive.currentMPS, -m_drive.currentMPS);

      else

      m_drive.tankDriveWithFeedforward(m_drive.currentMPS, m_drive.currentMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.currentMPS = 0;
    m_drive.tankDriveWithFeedforward(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
