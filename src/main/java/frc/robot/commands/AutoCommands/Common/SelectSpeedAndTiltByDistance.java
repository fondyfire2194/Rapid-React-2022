// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterRangeConstants;
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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_shooter.calculatedCameraDistance < 3)
      m_shooter.calculatedCameraDistance = 3;
    if (m_shooter.calculatedCameraDistance > 20)
      m_shooter.calculatedCameraDistance = 20;
    int distance = (int) m_shooter.calculatedCameraDistance;
    if (distance < ShooterRangeConstants.range1) {
      m_tilt.cameraCalculatedTiltPosition = ShooterRangeConstants.tiltRange1;
    } else if (distance >= ShooterRangeConstants.range1 && distance < ShooterRangeConstants.range2) {
      m_tilt.cameraCalculatedTiltPosition = ShooterRangeConstants.tiltRange2;
    } else if (distance >= ShooterRangeConstants.range2 && distance < ShooterRangeConstants.range3) {
      m_tilt.cameraCalculatedTiltPosition = ShooterRangeConstants.tiltRange3;
    } else if (distance >= ShooterRangeConstants.range3 && distance < ShooterRangeConstants.range4) {
      m_tilt.cameraCalculatedTiltPosition = ShooterRangeConstants.tiltRange4;
    } else {
      // out of range
      m_tilt.cameraCalculatedTiltPosition = ShooterRangeConstants.tiltRange4;
    }

    m_shooter.cameraCalculatedSpeed = m_shooter.rpmFromCameraDistance[(int) (m_shooter.calculatedCameraDistance - 1)];
    SmartDashboard.putNumber("calcTilt", m_tilt.cameraCalculatedTiltPosition);
    SmartDashboard.putNumber("calcRPM", m_shooter.cameraCalculatedSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
