// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldMap;
import frc.robot.Constants.FieldConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CalculateTargetDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private final RevTurretSubsystem m_turret;
  private final RawContoursV2 m_rcv2;

  private double m_cameraVerticalError;
  private double cameraAngle;

  public CalculateTargetDistance(LimeLight limelight, RawContoursV2 rcv2, RevTiltSubsystem tilt,
      RevTurretSubsystem turret,
      RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_tilt = tilt;
    m_shooter = shooter;
    m_turret = turret;
    m_rcv2 = rcv2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.calculatedCameraDistance = -1;
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

    if ((m_limelight.getIsTargetFound() || RobotBase.isSimulation())) {

      // m_cameraVerticalError = m_limelight.getdegVerticalToTarget() -
      // m_limelight.verticalOffset;
      m_cameraVerticalError = m_limelight.getdegRotationToTarget()- m_limelight.horizontalOffset;
      SmartDashboard.putNumber("CAMVE", m_cameraVerticalError);
      SmartDashboard.putNumber("HeightDiff", FieldConstants.heightDifference);
      SmartDashboard.putNumber("CamAng", cameraAngle);
    SmartDashboard.putNumber("CamOFM_LI", m_limelight.horizontalOffset);

      if (RobotBase.isSimulation()) {

        m_cameraVerticalError = 0;

      }

      double tanAngleSum = Math.tan((Math.toRadians(m_cameraVerticalError +
          cameraAngle))); 
           SmartDashboard.putNumber("TanSum", tanAngleSum);

      m_shooter.calculatedCameraDistance = (int) (FieldConstants.heightDifference /
          tanAngleSum);

      // m_shooter.calculatedCameraDistance = (int)
      // Math.round(FieldConstants.heightDifference

      // / (Math.tan(m_rcv2.getCenterTxAngle() *
      // Math.cos(m_rcv2.getCenterTyAngle()))));

      // m_shooter.calculatedCameraDistance += (FieldMap.visionStripRingDiameter / 2);
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
