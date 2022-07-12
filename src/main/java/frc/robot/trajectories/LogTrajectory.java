/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class LogTrajectory extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "TrajVel", "TrajAccel", "TrajCurv", "TrajX", "TrajY", "TrajDeg"
  };

  public static String[] units = { "Secs", "MPS", "MPSPS", "RadPerMeter", "Meters", "Meters", "Deg"
  };

  private int loopCtr;
  private boolean fileOpenNow;

  private final FondyFireTrajectory m_ff;
  private final Trajectory m_traj;

  private double logTime;
  private double firstLogTime;
  private double time;
  private String m_name;
  private double startTime;

  private int pointer;
  private int trajLength;

  private State trajState;

  public LogTrajectory(FondyFireTrajectory ff, Trajectory traj, String trajName) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_ff = ff;
    m_traj = traj;
    m_name = trajName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope;

    ope = m_ff.trajLogger.init(m_name, names, units);

    loopCtr = 0;
    fileOpenNow = false;
    pointer = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500 && !fileOpenNow) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data
    if (fileOpenNow)

      m_ff.trajLogInProgress = true;

    SmartDashboard.putBoolean("lip", m_ff.trajLogInProgress);

    trajLength = m_traj.getStates().size();

    trajState = m_traj.getStates().get(pointer);

    m_ff.trajLogger.writeData(

        trajState.timeSeconds,
        trajState.velocityMetersPerSecond,
        trajState.accelerationMetersPerSecondSq,
        trajState.curvatureRadPerMeter,

        trajState.poseMeters.getTranslation().getX(),
        trajState.poseMeters.getTranslation().getY(),
        trajState.poseMeters.getRotation().getDegrees()

    );

    pointer++;

    SmartDashboard.putNumber("ptr", pointer);
    SmartDashboard.putNumber("Leng", trajLength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_ff.trajLogger.close();
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pointer >= trajLength - 1;
  }
}
