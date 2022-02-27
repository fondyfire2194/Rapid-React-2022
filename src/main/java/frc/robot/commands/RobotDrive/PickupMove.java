// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;

public class PickupMove extends CommandBase {
  /** Creates a new PickupMove. */
  private final RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_speed;
  private double minSpeed = .2;
  private double currentSpeed;
  private double decelDistance;
  private double kp;
  private boolean plusDirection;
  private double remainingDistance;

  private double useSpeed;

  private int loopCtr;

  private IntakesSubsystem m_intake;

  private double yawInUse;

  private double gyroYawGain;

  private double intakeCameraYawGain = .1;

  public PickupMove(RevDrivetrain drive, IntakesSubsystem intake, double endpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_intake = intake;
    m_endpoint = endpoint;
    m_speed = Math.abs(speed);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    kp = m_speed / decelDistance;

    plusDirection = m_endpoint > m_drive.getAverageDistance();

    currentSpeed = m_speed;

    if (DriverStation.isTeleopEnabled()) {

      m_drive.logDriveItems = true;
    }

    m_drive.resetGyro();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * speed calculations are done in positive values and inverted based on
     * direction
     * 
     */

    remainingDistance = m_endpoint - m_drive.getLeftDistance();//

    if (remainingDistance < 0) {
      plusDirection = false;
    }

    currentSpeed = kp * Math.abs(remainingDistance);

    if (currentSpeed >= m_speed)
      currentSpeed = m_speed;
    if (currentSpeed < minSpeed)
      currentSpeed = minSpeed;

    useSpeed = currentSpeed;

    if (!plusDirection)

      useSpeed = -useSpeed;

    if (m_intake.useFrontIntake) {

      yawInUse = m_intake.getActiveTargetYaw() * intakeCameraYawGain;

    }

    else {

      yawInUse = m_drive.getYaw() * Pref.getPref("dRStKp");
    }

    m_drive.arcadeDrive(useSpeed, -yawInUse);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentSpeed = 0;
    m_drive.arcadeDrive(0, 0);
    m_drive.logDriveItems = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return plusDirection && m_drive.getLeftDistance() > m_endpoint
        || !plusDirection && m_drive.getLeftDistance() < m_endpoint;

  }
}
