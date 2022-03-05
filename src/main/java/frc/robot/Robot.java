/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.RotateLimelight90;
import frc.robot.commands.AutoCommands.AllRetPuShoot;
import frc.robot.commands.AutoCommands.PUS3;
import frc.robot.commands.AutoCommands.Taxi;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Vision.SetUpLimelightForDriver;
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
  public static BooleanLogEntry rrBooleanLog;
  public static DoubleLogEntry rrDoubleLog;
  public static StringLogEntry rrStringLog;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Starts recording to data log
    DataLogManager.start();

    // Set up custom log entries
    DataLog log = DataLogManager.getLog();
    rrBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    rrDoubleLog = new DoubleLogEntry(log, "/my/double");
    rrStringLog = new StringLogEntry(log, "/my/string");

    m_robotContainer = new RobotContainer();

    getAllianceColor();

    RotateLimelight90.init();

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
    if (tst >= 298) {
      // Only log when necessary
      rrBooleanLog.append(true);
      rrDoubleLog.append(3.5);
      rrStringLog.append("wow!");
    }

   SmartDashboard.putBoolean("FMSConn", m_robotContainer.isMatch);
    m_robotContainer.m_shooter.driverThrottleValue = m_robotContainer.getThrottle();
    // SmartDashboard.putNumber("thr",m_robotContainer.m_driverController.getThrottle());
    SmartDashboard.putNumber("thr1", m_robotContainer.getThrottle());
    SmartDashboard.putNumber("thry", m_robotContainer.m_driverController.getY());

    m_robotContainer.m_limelight.periodic();

    m_robotContainer.m_tilt.testLock = m_robotContainer.m_driverController.getTrigger();

    m_robotContainer.m_tilt.testLockFromThrottle = m_robotContainer.m_driverController.getThrottle();

    loopCtr++;

    SmartDashboard.putNumber("LPCTRA", loopCtr);

    m_robotContainer.m_intakes.twoCargoOnBoard = m_robotContainer.m_transport.getCargoAtShoot()

        && (m_robotContainer.m_intakes.getCargoAtFront() || m_robotContainer.m_intakes.getCargoAtRear());

    m_robotContainer.m_transport.cargoAvailable = m_robotContainer.m_transport.getCargoAtShoot()

        || m_robotContainer.m_intakes.getCargoAtFront() || m_robotContainer.m_intakes.getCargoAtRear();

    m_robotContainer.m_shooter.wrongCargoColor = getAllianceColor() && !m_robotContainer.m_transport.getCargoIsBlue()
        || !getAllianceColor() && !m_robotContainer.m_transport.getCargoIsRed();
    ;

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (m_robotContainer.m_drive.isStopped()) {
      m_robotContainer.m_drive.setIdleMode(false);
    }
    m_robotContainer.m_limelight.useVision = false;
    m_robotContainer.checkCANDevices();

    new SetUpLimelightForDriver(m_robotContainer.m_limelight).schedule();

  }

  @Override
  public void disabledPeriodic() {

  }

  public void autonomousInit() {

    m_robotContainer.m_limelight.allianceIsBlue = DriverStation.getAlliance() == Alliance.Blue;

    m_robotContainer.m_drive.setIdleMode(true);

    if (RobotBase.isReal())
      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    Shuffleboard.selectTab("Competition");

    Shuffleboard.startRecording();
    // get delay time

    m_startDelay = 0;// (double) m_robotContainer.m_setup.startDelayChooser.getSelected();

    autoChoice = 0;// m_robotContainer.m_setup.autoChooser.getSelected();

    LimeLight ll = m_robotContainer.m_limelight;
    RevTiltSubsystem tilt = m_robotContainer.m_tilt;
    RevTurretSubsystem turret = m_robotContainer.m_turret;
    RevShooterSubsystem shooter = m_robotContainer.m_shooter;
    RevDrivetrain drive = m_robotContainer.m_drive;
    CargoTransportSubsystem transport = m_robotContainer.m_transport;
    IntakesSubsystem intake = m_robotContainer.m_intakes;
    RawContoursV2 rcv2 = m_robotContainer.m_rCV2;
    Compressor comp = m_robotContainer.m_compressor;

    switch (autoChoice) {

      case 0:// taxi start anywhere as agreed with other teams inside a tarmac facing in

        m_autonomousCommand = new Taxi(m_robotContainer.m_drive, -(FieldMap.robotLength + 1));

        break;

      case 1:// in front of power port, move back use shooter data index 1

        m_autonomousCommand = new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, transport, comp,
            FieldMap.leftTarmacData);

        break;

      case 2://

        m_autonomousCommand = new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, transport, comp,
            FieldMap.rightTarmacData);

        break;

      case 3://

        m_autonomousCommand = new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, transport, comp,
            FieldMap.rightCenTarmacData);

        break;

      case 4://

        m_autonomousCommand = new PUS3(intake, drive, turret, tilt, ll, shooter, rcv2, transport, comp,
            FieldMap.rightCenTarmacData);

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

      // new SetLogItemsState(m_robotContainer.m_shooter, m_robotContainer.m_tilt,
      // m_robotContainer.m_turret, false)
      // .schedule();

    }

    Shuffleboard.update();

    Shuffleboard.startRecording();

    m_robotContainer.m_drive.setIdleMode(true);

    autoHasRun = false;

    if (RobotBase.isReal() && !m_robotContainer.m_tilt.positionResetDone)
      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    m_robotContainer.m_limelight.useVision = false;

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  public void teleopPeriodic() {

    // m_robotContainer.setupGamepad.setRumble(RumbleType.kLeftRumble, 1.0);

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

  private void setStartingPose(Pose2d pose) {

    m_robotContainer.m_drive.resetAll();

    m_robotContainer.m_drive.resetPose(pose);
  }

  public static boolean getAllianceColor() {

    return DriverStation.getAlliance() == Alliance.Blue;
  }

  public static boolean getFMConnected() {

    return DriverStation.isFMSAttached();
  }

}
