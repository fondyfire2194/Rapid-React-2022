/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.AutoCommands.RetPuAdvShoot;
import frc.robot.commands.AutoCommands.Common.SetShootPositionSpeedTilt;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int autoChoice;

  public RobotContainer m_robotContainer;
  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;
  public double timeToStart;
  int tst;
  private int loopCtr;
  // public static BooleanLogEntry rrBooleanLog;
  // public static DoubleLogEntry rrDoubleLog;
  // public static StringLogEntry rrStringLog;

  double[] data = { 0, 0, 0, 0, 0, 0, 0, 0 };

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Starts recording to data log
    // DataLogManager.start();

    // Set up custom log entries
    // DataLog log = DataLogManager.getLog();
    // rrBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    // rrDoubleLog = new DoubleLogEntry(log, "/my/double");
    // rrStringLog = new StringLogEntry(log, "/my/string");

    m_robotContainer = new RobotContainer();

    getAllianceColorBlue();

    if (m_robotContainer.isMatch)

      Shuffleboard.selectTab("Pre-Round");

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // m_robotContainer.m_setup.checkLimits();
    // if (tst >= 298) {
    // // Only log when necessary
    // rrBooleanLog.append(true);

    // rrDoubleLog.append(3.5);
    // rrStringLog.append("wow!");
    // }

    SmartDashboard.putBoolean("FMSConn", m_robotContainer.isMatch);
    m_robotContainer.m_shooter.driverThrottleValue = m_robotContainer.getThrottle();
    
    m_robotContainer.m_limelight.periodic();

    loopCtr++;

    SmartDashboard.putNumber("LPCTRA", loopCtr);

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    CommandScheduler.getInstance().cancelAll();
    if (m_robotContainer.m_drive.isStopped()) {
      m_robotContainer.m_drive.setIdleMode(false);
    }
    m_robotContainer.m_limelight.useVision = false;
    m_robotContainer.checkCANDevices();

  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("RetPos", FieldMap.leftTarmacData[0]);
  }

  public void autonomousInit() {

    m_robotContainer.m_drive.resetAll();
    autoHasRun = false;
    m_robotContainer.m_limelight.allianceIsBlue = DriverStation.getAlliance() == Alliance.Blue;

    m_robotContainer.m_drive.setIdleMode(true);

    if (RobotBase.isReal())

      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    Shuffleboard.selectTab("Competition");

    Shuffleboard.startRecording();
    // get delay time

    m_startDelay = m_robotContainer.m_preOi.startDelayChooser.getSelected();

    autoChoice = m_robotContainer.m_preOi.autoChooser.getSelected();

    LimeLight ll = m_robotContainer.m_limelight;
    RevTiltSubsystem tilt = m_robotContainer.m_tilt;
    RevTurretSubsystem turret = m_robotContainer.m_turret;
    RevShooterSubsystem shooter = m_robotContainer.m_shooter;
    RevDrivetrain drive = m_robotContainer.m_drive;
    CargoTransportSubsystem transport = m_robotContainer.m_transport;
    IntakesSubsystem intake = m_robotContainer.m_intake;
    RawContoursV2 rcv2 = m_robotContainer.m_rcv2;
    Compressor comp = m_robotContainer.m_compressor;

    switch (autoChoice) {

      case 0:
        m_autonomousCommand = new MessageCommand("Did Nothing Auto");
        break;

      case 1:// taxi start anywhere as agreed with other teams inside a tarmac facing in
        ll.setPipeline(PipelinesConstants.noZoom960720);
        m_autonomousCommand = new PositionStraight(m_robotContainer.m_drive, -1, .3);

        break;

      case 2:// left tarmac upper shoot
        data = FieldMap.leftTarmacData;
        ll.setPipeline(PipelinesConstants.noZoom960720);
        m_autonomousCommand = new RetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll, comp,
            data);

        break;

      case 3://
        data = FieldMap.rightTarmacData;
        ll.setPipeline(PipelinesConstants.noZoom960720);
        m_autonomousCommand = new RetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll, comp,
            data);
        break;

      case 4://
        data = FieldMap.centerTarmacData;
        m_autonomousCommand = new RetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll, comp,
            data);
        break;

      default:

        break;

    }

    startTime = Timer.getFPGATimestamp();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }

    m_robotContainer.m_setup.timeToStart = Math.round(startTime + m_startDelay - Timer.getFPGATimestamp());

    if (m_robotContainer.m_setup.timeToStart < 0)
      m_robotContainer.m_setup.timeToStart = 0;

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }
    autoHasRun = false;
    Shuffleboard.update();

    Shuffleboard.startRecording();

    m_robotContainer.m_drive.setIdleMode(true);

    autoHasRun = false;

    CommandScheduler.getInstance().cancelAll();

    if (RobotBase.isReal() && !m_robotContainer.m_tilt.positionResetDone)

      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);
  
      m_robotContainer.m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);
  
      m_robotContainer.m_limelight.useVision = false;

    m_robotContainer.m_intake.setFrontActive();

    m_robotContainer.m_shooter.shootSpeedSource = m_robotContainer.m_shooter.fromPreset;

    new SetShootPositionSpeedTilt(m_robotContainer.m_shooter, m_robotContainer.m_tilt, m_robotContainer.m_limelight, 0).schedule();;

    new CalculateTargetDistance(m_robotContainer.m_limelight, m_robotContainer.m_rcv2, m_robotContainer.m_tilt,
        m_robotContainer.m_turret, m_robotContainer.m_shooter).schedule();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  public void teleopPeriodic() {

    if (m_robotContainer.m_transport.wrongCargoColor) {

      m_robotContainer.m_transport.wrongCargoColor = false;

      new RunCargoOutShooter(m_robotContainer.m_shooter, m_robotContainer.m_intake, m_robotContainer.m_transport)
          .schedule();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static boolean getAllianceColorBlue() {

    return DriverStation.getAlliance() == Alliance.Blue;
  }

  public static boolean getFMSConnected() {

    return DriverStation.isFMSAttached();
  }

}
