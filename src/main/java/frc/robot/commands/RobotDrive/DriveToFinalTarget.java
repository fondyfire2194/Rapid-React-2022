// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.RawContoursV2;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class DriveToFinalTarget extends CommandBase {
  /** Creates a new DriveToFinalTarget. */
  private RawContoursV2 m_rcv2;
  private RevTurretSubsystem m_turret;
  private RevTiltSubsystem m_tilt;
  private RevDrivetrain m_drive;

  private double leftContourArea;
  private double rightContourArea;
  private double areaDifference;
  private PIDController lockController = new PIDController(.1, .001, 0);
  private boolean driveForward;

  public DriveToFinalTarget(RawContoursV2 rcv2, RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret, m_tilt, m_drive);
    m_rcv2 = rcv2;
    m_turret = turret;
    m_tilt = tilt;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftContourArea = m_rcv2.getLeftArea();
    rightContourArea = m_rcv2.getRightArea();

    if (m_turret.getLockAtTarget()) {
      if (m_turret.getAngle() >= 0) {

        driveForward = m_turret.getAngle() > 0 && leftContourArea > rightContourArea ||
            m_turret.getAngle() < 0 && rightContourArea > leftContourArea;
      }

      if (driveForward) {

        areaDifference = leftContourArea - rightContourArea;

      } else {

        areaDifference = rightContourArea - leftContourArea;
        
      }

      double pidOut = lockController.calculate(areaDifference, 0);

      m_drive.arcadeDrive(pidOut, 0);
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
