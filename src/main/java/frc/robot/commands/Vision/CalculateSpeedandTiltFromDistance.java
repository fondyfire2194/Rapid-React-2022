// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class CalculateSpeedandTiltFromDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;

  private LinearFilter m_filter = LinearFilter.movingAverage(5);

  public CalculateSpeedandTiltFromDistance( RevTiltSubsystem tilt,
      RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  /**
   * Calculates the final MPS for the shooter from the camera distance and camera
   * vertical error
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_shooter.useCameraSpeed) {

      m_shooter.requiredMps = m_shooter.RPMFromCameraDistance[(int) m_shooter.calculatedCameraDistance];

      m_tilt.cameraCalculatedTiltOffset = 4 - ((m_shooter.calculatedCameraDistance - 3) / 4);
    }

    else {
      m_shooter.cameraCalculatedSpeed = 0;

      m_tilt.cameraCalculatedTiltOffset = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
