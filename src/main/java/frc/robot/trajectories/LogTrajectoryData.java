/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class LogTrajectoryData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "TrajVel", "TrajDeg", "TrajAccel", "TrajCurv", "WheelLeftSpeed",
      "WheelRightSpeed", "LeftVolts", "RightVolts", "LeftRate", "RightRate", "GyroHdg" };
  public static String[] units = { "Secs", "MPS", "Deg", "MPSPS", "PCT", "MPS",
      "MPS", "Volt", "Volts", "MPS", "MPS", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;

  private final RevDrivetrain m_drive;
  private final FondyFireTrajectory m_ff;
  private final Trajectory m_traj;

  private double logTime;
  private double firstLogTime;
  private double time;
  private String m_name;

  public LogTrajectoryData(RevDrivetrain drive, FondyFireTrajectory ff, Trajectory traj, String trajName) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_ff = ff;
    m_traj = traj;
    m_name = trajName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    int ope = m_ff.trajLogger.init( m_name, names, units);
    SmartDashboard.putNumber("OPE", ope);
    SmartDashboard.putString("name", m_name);
  
    loopCtr = 0;
    fileOpenNow = false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data every 100ms
    if (fileOpenNow)
      m_ff.trajLogInProgress = true;

    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (m_ff.logTrajItems && Timer.getFPGATimestamp() > logTime + .1) {
      logTime = Timer.getFPGATimestamp();

      if (firstLogTime == 0)
        firstLogTime = logTime;

      time = Timer.getFPGATimestamp() - firstLogTime;

      m_ff.trajLogger.writeData(
          time,
          m_traj.sample(time).velocityMetersPerSecond,
          m_traj.sample(time).poseMeters.getRotation().getDegrees(),
          m_traj.sample(time).accelerationMetersPerSecondSq,
          m_traj.sample(time).curvatureRadPerMeter,

          m_drive.getWheelSpeeds().leftMetersPerSecond,
          m_drive.getWheelSpeeds().rightMetersPerSecond,

          m_drive.leftVolts,
          m_drive.rightVolts,
          m_drive.getLeftRate(),
          m_drive.getRightRate(),
          m_drive.getHeading());

    }

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
    return time > m_traj.getTotalTimeSeconds();
  }
}
