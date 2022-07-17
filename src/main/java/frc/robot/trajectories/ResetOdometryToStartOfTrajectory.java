// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometryToStartOfTrajectory extends InstantCommand {
  private FondyFireTrajectory m_fftraj;
  private Trajectory m_traj;
  private RevDrivetrain m_drive;

  public ResetOdometryToStartOfTrajectory(FondyFireTrajectory fftraj, Trajectory traj, RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_fftraj = fftraj;
    m_traj = traj;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drive.resetOdometry(m_traj.getInitialPose());

   
    m_drive.trajectoryRunning = true;

  }
}
