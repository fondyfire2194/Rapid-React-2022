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
    private final FondyFireTrajectory m_fftraj;
    private final String m_name;

    Translation2d translation;
    State trajState;
    double startTime;
    double time;
    double trajStepTime;
    private double lastStepTime;

    public NetTablesLog(RevDrivetrain drive, FondyFireTrajectory fftraj, Trajectory traj, String name) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drive = drive;
        m_fftraj = fftraj;
        m_traj = traj;
        m_name = name;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        m_fftraj.trajName.setString(m_name);
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
            m_fftraj.sampleTime.setNumber(time);
            m_fftraj.trajVel.setNumber(trajState.velocityMetersPerSecond);
            m_fftraj.trajAcc.setNumber(trajState.accelerationMetersPerSecondSq);
            m_fftraj.trajCurv.setNumber(trajState.curvatureRadPerMeter);

            m_fftraj.trajX.setNumber(trajState.poseMeters.getTranslation().getX());
            m_fftraj.trajY.setNumber(trajState.poseMeters.getTranslation().getY());
            m_fftraj.trajRot.setNumber(trajState.poseMeters.getRotation().getDegrees());

            translation = m_drive.mOdometry.getPoseMeters().getTranslation();
            m_fftraj.m_xEntry.setNumber(translation.getX());
            m_fftraj.m_yEntry.setNumber(translation.getY());

            m_fftraj.leftMeasurement.setNumber(m_drive.getWheelSpeeds().leftMetersPerSecond);
            m_fftraj.leftReference.setNumber(m_drive.leftController.getSetpoint());
            m_fftraj.leftError.setNumber(
                    m_drive.leftController.getSetpoint() -
                            m_drive.getWheelSpeeds().leftMetersPerSecond);

            m_fftraj.rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
            m_fftraj.rightReference.setNumber(m_drive.rightController.getSetpoint());
            m_fftraj.rightError.setNumber(
                    m_drive.rightController.getSetpoint() -
                            m_drive.getWheelSpeeds().rightMetersPerSecond);

            m_fftraj.angleMeasurement.setNumber(m_drive.getHeading());
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
