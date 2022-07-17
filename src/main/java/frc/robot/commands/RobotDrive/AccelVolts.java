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

  double maxMPS = 4;

  double voltsIncrementPer20ms;

  double maxVolts;

  double startMPSCommand = .2;

  private double voltsIncremementPerSecond;

  private boolean accPulse;

  private boolean decPulse;

  private int loopCtr;

  public AccelVolts(RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drive.currentMPSCommand = startMPSCommand;

    loopCtr = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopCtr++;

    if (loopCtr > 10 && !accPulse) {

      accPulse = true;
    }
    if (loopCtr > 20) {
      accPulse = false;
      loopCtr = 0;
    }

    if (accPulse) {

      m_drive.currentVolts = DriveConstants.kvVoltSecondsPerMeter * m_drive.currentMPSCommand + DriveConstants.ksVolts

          + DriveConstants.kaVoltSecondsSquaredPerMeter * accelMPSPSPS;
    }

    if (!accPulse)

    {
      m_drive.currentMPSCommand = startMPSCommand;
      m_drive.currentVolts = DriveConstants.kvVoltSecondsPerMeter * m_drive.currentMPSCommand + DriveConstants.ksVolts;
    }

    m_drive.tankDriveVolts(m_drive.currentVolts, m_drive.currentVolts);

  }// kaVoltSecondsSquaredPerMeter V / m/s/s

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
