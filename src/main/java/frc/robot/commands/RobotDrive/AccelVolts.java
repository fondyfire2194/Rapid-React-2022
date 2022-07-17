// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

public class AccelVolts extends CommandBase {
  /** Creates a new RunAtVolts. */
  private RevDrivetrain m_drive;

  double minVolts = 0;

  double accelMPSPSPS = .1;

  double maxMPS = 3;

  double voltsPerMPS;

  double voltsIncrementPer20ms;

  double maxVolts;

  double kvVoltSecondsPerMeter = 3.9;

  private double voltsIncremementPerSecond;

  public AccelVolts(RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.currentVolts = 0;

    m_drive.ksVolts = .23;

    voltsPerMPS = maxMPS / 12;

    voltsIncremementPerSecond = accelMPSPSPS / voltsPerMPS;

    SmartDashboard.putNumber("VIPS", voltsIncremementPerSecond);

    voltsIncrementPer20ms = voltsIncremementPerSecond / 50;

    maxVolts = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.currentVolts += voltsIncrementPer20ms;

    if (m_drive.currentVolts >= maxVolts)

      m_drive.currentVolts = maxVolts;

    m_drive.tankDriveVolts(m_drive.currentVolts, m_drive.currentVolts);

    m_drive.kvVolts = DriveConstants.kvVoltSecondsPerMeter * m_drive.getLeftRate();

    m_drive.accelerationVolts = m_drive.currentVolts - m_drive.ksVolts - m_drive.kvVolts;

    m_drive.voltspwermpssqd = m_drive.accelerationVolts / accelMPSPSPS;

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
