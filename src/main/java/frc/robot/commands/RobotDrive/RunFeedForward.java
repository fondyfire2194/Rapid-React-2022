// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class RunFeedForward extends CommandBase {
  /** Creates a new RunAtMPS. */
  private RevDrivetrain m_drive;

  

  private boolean m_minmpsus;

 double maxmps=3.5;
 double minmps =0;

  public RunFeedForward(RevDrivetrain drive, boolean minmpsus) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_minmpsus = minmpsus;
    

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.currentMPSCommand = 0;
    m_drive.resetEncoders();
    m_drive.resetGyro();
 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double mpsRange = maxmps - minmps;

    m_drive.currentMPSCommand = minmps + mpsRange * m_drive.throttleValue;

   
    if (m_minmpsus)

      m_drive.tankDriveWithFeedforward(-m_drive.currentMPSCommand, -m_drive.currentMPSCommand);

    else

      m_drive.tankDriveWithFeedforward(m_drive.currentMPSCommand, m_drive.currentMPSCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.currentMPSCommand = 0;
    m_drive.tankDriveWithFeedforward(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
