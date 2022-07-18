// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

import edu.wpi.first.math.trajectory.Trajectory.State;

public class NetTablesLog extends CommandBase {
  /** Creates a new NetTablesLog. */

  private final RevDrivetrain m_drive;
  private final Trajectory m_traj;
  private final String m_name;

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  NetworkTableEntry leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("left_reference");
  NetworkTableEntry sampleTime = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("sampleTime");

  NetworkTableEntry leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("left_measurement");
  NetworkTableEntry leftError = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("left_error");

  NetworkTableEntry rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("right_reference");
  NetworkTableEntry rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("right_measurement");
  NetworkTableEntry rightError = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("right_error");

  NetworkTableEntry angleMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("angle_measurement");

  NetworkTableEntry leftV = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("left_volts");

  NetworkTableEntry rightV = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("right_volts");

  NetworkTableEntry xPosn = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("x_posn");

  NetworkTableEntry yPosn = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("y_posn");

  NetworkTableEntry trajVel = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_vel");
  NetworkTableEntry trajAcc = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_acc");
  NetworkTableEntry trajCurv = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_curv");

  NetworkTableEntry trajX = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_x");
  NetworkTableEntry trajY = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_y");
  NetworkTableEntry trajRot = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_rot");

  NetworkTableEntry trajName = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("traj_name");

  Translation2d translation;
  State trajState;
  double startTime;
  double time;
  double trajStepTime;
  private double lastStepTime;

  public NetTablesLog(RevDrivetrain drive, Trajectory traj, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_traj = traj;
    m_name = name;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    trajName.setString(m_name);
    lastStepTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    time = Timer.getFPGATimestamp() - startTime;

    SmartDashboard.putNumber("Time", time);

    trajState = m_traj.sample(time);

    trajStepTime = trajState.timeSeconds;

    if (trajStepTime != lastStepTime) {

      lastStepTime = trajStepTime;
      sampleTime.setNumber(time);
      trajVel.setNumber(trajState.velocityMetersPerSecond);
      trajAcc.setNumber(trajState.accelerationMetersPerSecondSq);
      trajCurv.setNumber(trajState.curvatureRadPerMeter);

      trajX.setNumber(trajState.poseMeters.getTranslation().getX());
      trajY.setNumber(trajState.poseMeters.getTranslation().getY());
      trajRot.setNumber(trajState.poseMeters.getRotation().getDegrees());

      translation = m_drive.mOdometry.getPoseMeters().getTranslation();
      m_xEntry.setNumber(translation.getX());
      m_yEntry.setNumber(translation.getY());

      leftMeasurement.setNumber(m_drive.getWheelSpeeds().leftMetersPerSecond);
      leftReference.setNumber(m_drive.leftController.getSetpoint());
      leftError.setNumber(
          m_drive.leftController.getSetpoint() -
              m_drive.getWheelSpeeds().leftMetersPerSecond);

      rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
      rightReference.setNumber(m_drive.rightController.getSetpoint());
      rightError.setNumber(
          m_drive.rightController.getSetpoint() -
              m_drive.getWheelSpeeds().rightMetersPerSecond);

      angleMeasurement.setNumber(m_drive.getHeading());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time > m_traj.getTotalTimeSeconds();
  }
}
