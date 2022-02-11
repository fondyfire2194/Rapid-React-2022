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
import frc.robot.AngleSolver;
import frc.robot.GetTarget;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LogHubTarget extends CommandBase {
  /**
   * Creates a new LogHubData.
   */
  public final String[] names = { "LAr", "CAr", "RAr", "TAr",
      "LAn", "CAn", "RAn", "TAn",
      "TuAn", "TiAn", "Cx", "CaAn","ASAn","Dist"
  };

  public static String[] units = {
      "A", "A", "A", "A",
      "Dg", "Dg", "Dg", "Dg",
      "Dg", "Dg", "Px", "Dg","Dg","Mtr"
  };

  private int loopCtr;
  private boolean fileOpenNow;

  private RevTiltSubsystem m_tilt;
  private RevTurretSubsystem m_turret;

  private LimeLight m_ll;
  private AngleSolver m_as;
  private GetTarget m_gt;

  private double logTime;

  private double firstLogTime;

  public LogHubTarget(AngleSolver as, GetTarget gt,
      RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight ll) {

    // Use addRequirements() here to declare subsystem dependencies.

    m_as = as;

    m_tilt = tilt;
    m_turret = turret;

    m_gt = gt;
    m_ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int opeH = m_as.hubLogger.init("TestRun", "Hub", names, units);
    SmartDashboard.putNumber("OPEHH", opeH);
    loopCtr = 0;
    fileOpenNow = false;
    m_as.setCLock3Contours(true);
    m_as.endLog = false;
    m_as.logInProgress = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow 1 second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;

    }
    // log data every 20ms
    if (fileOpenNow)
      m_as.logInProgress = true;

    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (firstLogTime == 0)
      firstLogTime = logTime;

    if (m_as.logInProgress && Timer.getFPGATimestamp() > logTime + .1) {

      logTime = Timer.getFPGATimestamp();

      m_as.hubLogger.writeData(

          m_as.getLeftArea(), m_as.getCenterArea(),
          
          m_as.getRightArea(), m_as.getTestTargetArea(),

          m_as.getLeftTxAngle(), m_as.getCenterTxAngle(), m_as.getRightTxAngle(),

          m_as.getTestTxAngle(), m_turret.getAngle(), m_tilt.getAngle(),

          m_gt.getTargetX(), m_gt.getTargetAngle(),

          m_as.averageAgleShift,m_as.leftDistance
          
          );


    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sdH = m_as.hubLogger.close();
    m_as.endLog = false;
    m_as.logInProgress = false;
    m_as.setCLock3Contours(false);

    SmartDashboard.putNumber("CloseHH", sdH);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_as.endLog;
  }
}
