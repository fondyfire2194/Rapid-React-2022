// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class RunAtVolts extends CommandBase {
  /** Creates a new RunAtVolts. */
  private RevDrivetrain m_drive;
  double maxVolts = 5;
  double minVolts = 0;

  public RunAtVolts(RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double voltsRange = maxVolts - minVolts;

    m_drive.currentVolts = minVolts + voltsRange * m_drive.voltsValue;
    m_drive.tankDriveVolts(m_drive.currentVolts, m_drive.currentVolts);
    m_drive.leftmpspervolt = m_drive.currentVolts / m_drive.getLeftRate();
    m_drive.rightmpspervolt = m_drive.currentVolts / m_drive.getRightRate();
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
