// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This is the default command. It will lock onto the vision target if seen otherwise it will hold where it is
 * 
 * It teleop the turret can be set by the driver to either straight ahead or slightly left.
 * 
 * The tilt will be set to minimum angle if driving straight.
 * The further away the target is, the lower the tilt needs to be so it should pick up the target and lift as the robot gets closer.
 * 
 *  If using the trench then the turret will be set slightly laeft and the tilt higher. The target wont be seen as quickly.
 * 
 * The driver can move the tilt up t look for the target if it isn't picked up.
 */

package frc.robot.commands.Turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionHoldTurret extends CommandBase {
  /** Creates a new Position Hold Turret. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private boolean targetSeen;
  private double cameraHorizontalError;

  private Debouncer visionTargetDebounce = new Debouncer(.025, DebounceType.kBoth);

  public PositionHoldTurret(RevTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_turret.programRunning = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetSeen = m_limelight.getIsTargetFound();

    // To limit effect of possible Limelight jitter, valid target is held off for 25
    // ms and will be held valid for 25 ms after if is lost. During the delay
    // time after it is lost, the previous value of error will be used

    m_turret.validTargetSeen = m_limelight.useVision && visionTargetDebounce.calculate(targetSeen);

    if (targetSeen && m_turret.validTargetSeen) {

      cameraHorizontalError = m_limelight.getdegRotationToTarget();
    }

    if (!targetSeen && !m_turret.validTargetSeen) {

      m_turret.goToPosition(m_turret.targetAngle);

      cameraHorizontalError = 0;
    }

    else {

      m_turret.lockTurretToVision(cameraHorizontalError);

      m_turret.targetAngle = m_turret.getAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.targetAngle = m_turret.getAngle();
    m_turret.validTargetSeen = false;
    m_turret.visionOnTarget = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
