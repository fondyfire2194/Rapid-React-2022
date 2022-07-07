// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlotTrajectory extends InstantCommand {
  private RevDrivetrain m_drive;
  private Trajectory m_traj;

  public PlotTrajectory(RevDrivetrain drive, Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_traj = traj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.plotTrajectory(m_traj);
  }
}
