// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreateTrajectory extends InstantCommand {
  private RevDrivetrain m_drive;
  private FondyFireTrajectory m_fftraj;
  private Trajectory m_traj;

  public CreateTrajectory(RevDrivetrain drive, FondyFireTrajectory fftraj, Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_fftraj = fftraj;
    m_traj = traj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_traj = TrajectoryGenerator.generateTrajectory(

        m_drive.getPose(),
        List.of(),
        m_fftraj.centerHideOppCargo,
        m_fftraj.withSpeedAndAcceleration(1, .25)
            .setReversed(false));

  }
}
