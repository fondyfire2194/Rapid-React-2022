// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.RevTurretSubsystem;

public class WaitTurretLocked extends CommandBase {
  /** Creates a new WaitTiltTurretLocked. */
  private RevTurretSubsystem m_turret;
  private LimeLight m_limelight;

  public WaitTurretLocked( RevTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_turret = turret;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return

    m_turret.validTargetSeen && m_limelight.getHorOnTarget(m_turret.turretVisionTolerance);
  }
}
