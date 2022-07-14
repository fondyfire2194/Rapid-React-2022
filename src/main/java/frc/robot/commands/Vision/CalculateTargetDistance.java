// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;

public class CalculateTargetDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  private final LimeLight m_limelight;

  private final RevShooterSubsystem m_shooter;

  private double m_cameraVerticalError;
  private double cameraAngle;
  private boolean useThrottle = false;

  public CalculateTargetDistance(LimeLight limelight, RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;

    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.calculatedCameraDistance = -9999;
    cameraAngle = FieldConstants.CAMERA_ANGLE;
  }

  /*
   * *
   * Target distance can be calculated from
   * 
   * tan(a1+a2) = (h2-h1) / d so d = (h2-h1) / tan(a1+a2)
   * 
   * where h1 is the camera height, h2 is the target height
   * 
   * a1 is the camera angle and a2 is the vertical angle the camera reports of the
   * target
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (useThrottle) {
      m_shooter.runContinuous = true;
      double maxdistance = 20;
      double mindist = 2;
      double rpmRange = maxdistance - mindist;

      m_shooter.calculatedCameraDistance = mindist + rpmRange * m_shooter.driverThrottleValue;

    }

    else if ((m_limelight.getIsTargetFound())) {

      // m_cameraVerticalError = m_limelight.getdegVerticalToTarget() -
      // m_limelight.verticalOffset;
      m_cameraVerticalError = -m_limelight.getdegVerticalToTarget();// - m_limelight.horizontalOffset;
      // SmartDashboard.putNumber("CAMVE", m_cameraVerticalError);
      // SmartDashboard.putNumber("HeightDiff", FieldConstants.heightDifference);
      // SmartDashboard.putNumber("CamAng", cameraAngle);
      // SmartDashboard.putNumber("CamOFM_LI", m_limelight.horizontalOffset);

      if (RobotBase.isSimulation()) {

        m_cameraVerticalError = 0;

      }

      double tanAngleSum = Math.tan((Math.toRadians(m_cameraVerticalError +
          cameraAngle)));
      SmartDashboard.putNumber("TanSum", tanAngleSum);

      m_shooter.calculatedCameraDistance = FieldConstants.heightDifference /
          tanAngleSum;

      m_shooter.calculatedCameraDistance = m_shooter.calculatedCameraDistance - FieldConstants.CAMERA_TO_FRONT_BUMPER;

      if (m_shooter.calculatedCameraDistance < 0)
        m_shooter.calculatedCameraDistance = 5;

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
