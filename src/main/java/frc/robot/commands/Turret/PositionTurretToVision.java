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
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private double m_endpoint;
  private int loopCtr;
  private final int filterCount = 3;
  private int visionFoundCounter;
  private boolean targetSeen;
  boolean endIt;
  private int correctionCtr;
  private boolean lookForTarget;
  private double remainingDistance;

  public PositionTurretToVision(RevTurretSubsystem turret, LimeLight limelight, double endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_endpoint = endpoint;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.programRunning = 3;
    m_turret.targetAngle = m_endpoint;
    targetSeen = false;
    visionFoundCounter = 0;
    loopCtr = 0;
    m_limelight.setHorizontalOffset(m_turret.targetHorizontalOffset);
    m_turret.turretUseVision = false;
    m_limelight.useVision = false;
    m_limelight.setLEDMode(LedMode.kpipeLine);
    m_turret.correctedEndpoint = m_endpoint;
    lookForTarget = false;

    visionFoundCounter = filterCount;
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * 
   * 
   *
   * 
   * A filter ignores loss of target for 100ms
   * 
   * 
   */
  @Override
  public void execute() {

    loopCtr++;

    remainingDistance = Math.abs(m_endpoint - m_turret.getAngle());

    targetSeen = remainingDistance < 5 && m_limelight.getIsTargetFound();

    if (targetSeen && !m_turret.validTargetSeen && visionFoundCounter < filterCount) {

      visionFoundCounter++;
    }

    if (!m_turret.validTargetSeen && visionFoundCounter >= filterCount) {

      m_turret.validTargetSeen = true;
    }

    if (m_turret.validTargetSeen) {

      m_turret.correctedEndpoint = (m_turret.getAngle() - m_limelight.getdegRotationToTarget()

          - m_turret.targetHorizontalOffset);

      m_turret.targetAngle = m_turret.correctedEndpoint;

    }

    if (!targetSeen && m_turret.validTargetSeen) {

      visionFoundCounter--;
    }

    if (!targetSeen && m_turret.validTargetSeen && visionFoundCounter < 0) {

      visionFoundCounter = 0;

      m_turret.validTargetSeen = false;

    }

    m_turret.goToPositionMotionMagic(m_turret.targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.logTurretItems = false;
    m_turret.turretUseVision = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.getHorOnTarget(3) || m_turret.atTargetAngle() && loopCtr > 5 || loopCtr > 250;
  }
}
