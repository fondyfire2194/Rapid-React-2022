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
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class LogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "ElTime",

      "IsShooting", "ActSpeed", "L_Amps", "R_Amps",

      "AtSpeed", "Top RPM",

      "CargoAtShoot", "LatchAtShoot", "LowerRPM ", "AtSpeed",

      "CargoAtFront", "CargoAtRear", "FrontMotor", "RearMotor"

  };

  public static String[] units = { "Seconds", "Seconds",

      "T/F", "RPM", "Amps", "Amps",

      "T/F", "RPM",

      "T/F", "T/F", "RPM", "T/F",

      "T/F", "T/F", "Pct", "Pct" };

  private int loopCtr;
  private boolean fileOpenNow;

  private final RevShooterSubsystem m_shooter;
  private final CargoTransportSubsystem m_transport;
  private final IntakesSubsystem m_intake;

  private double logTime;
  private double firstLogTime;

  public LogShootData(RevShooterSubsystem shooter, CargoTransportSubsystem transport, IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope1 = m_shooter.shootLogger.init("TestRun", "Shoot", names, units);
    SmartDashboard.putNumber("OPE1", ope1);
    loopCtr = 0;
    fileOpenNow = false;
    logTime = 0;

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

      m_shooter.shootLogInProgress = true;

    if (logTime == 0)

      logTime = Timer.getFPGATimestamp();

    if (m_shooter.logShooterItems && Timer.getFPGATimestamp() > logTime + .1) {

      logTime = Timer.getFPGATimestamp();

      if (firstLogTime == 0)

        firstLogTime = logTime;

      m_shooter.shootLogger.writeData(

          logTime, logTime - firstLogTime,

          m_shooter.getShooterIsShooting() ? 1.0 : 0.0,
          m_shooter.getRPM(),
          m_shooter.getLeftAmps(),
          m_shooter.getRightAmps(),
          m_shooter.getShooterAtSpeed() ? 1.0 : 0.0,
          m_shooter.getTopRPM(),

          m_transport.getCargoAtShoot() ? 1.0 : 0.0,
          m_transport.latchCargoAtShoot ? 1.0 : 0.0,
          m_transport.getLowerRPM(),
          m_transport.getLowerRollerAtSpeed() ? 1.0 : 0.0,

          m_intake.getCargoAtFront() ? 1.0 : 0.0,
          m_intake.getCargoAtRear() ? 1.0 : 0.0,
          m_intake.getFrontMotor(),
          m_intake.getRearMotor());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_shooter.shootLogger.close();
    m_shooter.endShootFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endShootFile;
  }
}
