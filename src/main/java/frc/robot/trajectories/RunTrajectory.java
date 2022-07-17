// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectory extends SequentialCommandGroup {
  /** Creates a new RunTraj. */
  public RunTrajectory(FondyFireTrajectory fftraj, RevDrivetrain drive, Trajectory traj) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new ResetOdometryToStartOfTrajectory(drive, traj),

        fftraj.getRamsete(traj)

            .andThen(() -> drive.tankDriveVolts(0, 0)));

  }
}