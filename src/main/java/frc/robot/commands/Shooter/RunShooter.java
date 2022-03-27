/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RevShooterSubsystem;

public class RunShooter extends CommandBase {
  /**
   * Creates a new StartShooter.
   */
  private RevShooterSubsystem m_shooter;

  private double m_rpm;
  private double m_startTime;

  public RunShooter(RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_startTime = Timer.getFPGATimestamp();

    m_rpm = m_shooter.getRPMFromSpeedSource();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_rpm != m_shooter.getRPMFromSpeedSource()) {

      m_rpm = m_shooter.getRPMFromSpeedSource();
    }

    m_shooter.runShooterPlusRoller(m_rpm);

    m_shooter.isAtSpeed = Timer.getFPGATimestamp() > m_startTime + .75;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
