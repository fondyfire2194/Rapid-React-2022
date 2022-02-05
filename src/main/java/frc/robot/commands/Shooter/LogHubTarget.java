/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.RawContoursV2;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LogHubTarget extends CommandBase {
  /**
   * Creates a new LogHubData.
   */
  public final String[] names = { "Time", "ElTime","TargetSeen" ,"LeftArea", "CenterArea",
      "RightArea", "TargetArea", "LeftTx",
      "CenterTx", "RightTx", "TargetTx",
      "TurretAngle", "TiltAngle", "LeftRate"
  };

  public static String[] units = { "Sec", "Sec", "T/F",
  "Pct", "Pct", "Pct", "Pct",
   "Px", "Px", "Px", "Px", 
   "Deg" ,"Deg","MperS"};

  private int loopCtr;
  private boolean fileOpenNow;

  private RevTiltSubsystem m_tilt;
  private RevTurretSubsystem m_turret;
  private RawContoursV2 m_rcv2;
  private RevShooterSubsystem m_shooter;
  private RevDrivetrain m_drive;
  private LimeLight m_ll;

  private double logTime;

  private double firstLogTime;

  public LogHubTarget(RawContoursV2 rcv2, RevTiltSubsystem tilt,
      RevTurretSubsystem turret, RevShooterSubsystem shooter,
       RevDrivetrain drive, LimeLight ll) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_rcv2 = rcv2;
    m_tilt = tilt;
    m_turret = turret;
    m_shooter = shooter;
    m_drive = drive;
    m_ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope1 = m_shooter.hubTargetLogger.init("TestRun", "Shoot", names, units);
    SmartDashboard.putNumber("OPE1", ope1);
    loopCtr = 0;
    fileOpenNow = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double llht=0;

if(m_ll.getIsTargetFound())llht=1;

    // allow 1 second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;

    }
    // log data every shot
    if (fileOpenNow)
      m_shooter.hubLogInProgress = true;
    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (firstLogTime == 0)
      firstLogTime = logTime;

    if (m_shooter.log && Timer.getFPGATimestamp() > logTime + .1) {

      logTime = Timer.getFPGATimestamp();

      m_shooter.hubLogger.writeData(logTime, logTime - firstLogTime,llht,
          m_rcv2.getLeftArea(), m_rcv2.getCenterArea(),
          m_rcv2.getRightArea(), m_rcv2.getTestTargetArea(),
          m_rcv2.getLeftTx(), m_rcv2.getCenterTx(),
          m_rcv2.getRightTx(), m_rcv2.getTestTargetTx(),
          m_turret.getAngle(), m_tilt.getAngle(), m_drive.getLeftRate());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.endHubFile = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endHubFile;
  }
}
