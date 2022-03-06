// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class SelectSpeedAndTiltByDistance extends CommandBase {
  private RevShooterSubsystem m_shooter;
  private RevTiltSubsystem m_tilt;
  /** Creates a new SelectSpeedAndTiltByDistance. */
  public SelectSpeedAndTiltByDistance(RevShooterSubsystem shooter, RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.cameraCalculatedSpeed = m_shooter.rpmFromCameraDistance[m_shooter.calculatedCameraDistance];
    m_tilt.cameraCalculatedTiltPosition = m_shooter.tiltAngleFromCamerDistance[m_shooter.calculatedCameraDistance];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
