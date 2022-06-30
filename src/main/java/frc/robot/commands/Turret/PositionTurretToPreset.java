// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretToPreset extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;

  boolean endIt;
  private int loopCtr;

  public PositionTurretToPreset(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
  

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_turret.targetAngle = m_turret.presetPosition;

    if (m_turret.targetAngle < -10) {

      m_turret.targetAngle = 0;

    }
    if (m_turret.targetAngle > 10) {

      m_turret.targetAngle = 0;

    }
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    m_turret.goToPosition(m_turret.targetAngle);

    loopCtr++;

    endIt = m_turret.atTargetAngle() && loopCtr > 10 && Math.abs(m_turret.getSpeed()) < 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (loopCtr > 10 && !endIt)

      m_turret.targetAngle = m_turret.getAngle();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
