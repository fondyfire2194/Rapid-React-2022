/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.LRetPuShoot;
import frc.robot.commands.AutoCommands.LRetPuShootLow;
import frc.robot.commands.AutoCommands.RRetCenPuShoot;
import frc.robot.commands.AutoCommands.RRetPuS3;
import frc.robot.commands.AutoCommands.RRetPuShoot;
import frc.robot.commands.AutoCommands.RRetPuShootLow;
import frc.robot.commands.AutoCommands.Taxi;
import frc.robot.commands.Shooter.ChooseShooterSpeedSource;
import frc.robot.commands.Shooter.SetLogItemsState;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Vision.CalculateSpeedFromDistance;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.commands.Vision.SetUpLimelightForDriver;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import edu.wpi.first.wpilibj.Compressor;

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

  private double canCheckWait;

  public RobotContainer m_robotContainer;
  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;
  public double timeToStart;
  int tst;
  private int loopCtr;
  private boolean allianceColorBlue;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    // CameraServer.startAutomaticCapture("Intake", 0);

    allianceColorBlue = DriverStation.getAlliance() == Alliance.Blue;
    if (allianceColorBlue) {
      m_robotContainer.m_intakes.activeCargoPipeline = m_robotContainer.m_intakes.blueCargoPipeline;
      m_robotContainer.m_intakes.activeLaunchPadPipeline = m_robotContainer.m_intakes.blueLaunchPadPipeline;
      m_robotContainer.m_intakes.setActivePipeline(m_robotContainer.m_intakes.activeCargoPipeline);

    }

    if (m_robotContainer.isMatch)
      Shuffleboard.selectTab("Pre-Round");

    // if (Pref.getPref("LogTilt") == 1.)
    // new LogTiltData(m_robotContainer.m_tilt,
    // m_robotContainer.m_limelight).schedule(true);

    // if (Pref.getPref("LogTurret") == 1.)
    // new LogTurretData(m_robotContainer.m_turret,
    // m_robotContainer.m_limelight).schedule(true);

    // if (Pref.getPref("LogShoot") == 1.)
    // new LogShootData(m_robotContainer.m_shooter, m_robotContainer.m_transport,
    // null).schedule(true);

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
    m_robotContainer.m_rCV2.getRawContourData();
    m_robotContainer.m_as.runValues();
    // m_robotContainer.m_getTarget.runCalcs();
    m_robotContainer.m_shooter.driverThrottleValue = m_robotContainer.getThrottle();
    // SmartDashboard.putNumber("thr",m_robotContainer.m_driverController.getThrottle());
    SmartDashboard.putNumber("thr1", m_robotContainer.getThrottle());
    SmartDashboard.putNumber("thry", m_robotContainer.m_driverController.getY());

    // m_robotContainer.m_limelight.periodic();

    // m_robotContainer.m_tilt.testLock =
    // m_robotContainer.m_driverController.getTrigger();

    // m_robotContainer.m_tilt.testLockFromThrottle =
    // m_robotContainer.m_driverController.getThrottle();
    loopCtr++;
    SmartDashboard.putNumber("LPCTRA", loopCtr);
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

    // run can check on switch
    // if (m_robotContainer.m_setup.runCan.getBoolean(false)) {
    // m_robotContainer.m_setup.checkCANDevices();
    // m_robotContainer.m_setup.runCan.setBoolean(false);
    // }
  }

  public void autonomousInit() {

    m_robotContainer.m_limelight.allianceIsBlue = DriverStation.getAlliance() == Alliance.Blue;

    m_robotContainer.m_drive.setIdleMode(true);

    if (RobotBase.isReal())
      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    new CalculateTargetDistance(m_robotContainer.m_limelight, m_robotContainer.m_tilt, m_robotContainer.m_turret,
        m_robotContainer.m_shooter).schedule(true);

    new CalculateSpeedFromDistance(m_robotContainer.m_limelight, m_robotContainer.m_tilt, m_robotContainer.m_turret,
        m_robotContainer.m_shooter).schedule(true);

    new ChooseShooterSpeedSource(m_robotContainer.m_shooter, m_robotContainer.m_tilt, m_robotContainer.m_turret, 0)
        .schedule(true);

    Shuffleboard.selectTab("Competition");
    Shuffleboard.startRecording();
    // get delay time

    m_startDelay = 0;// (double) m_robotContainer.m_setup.startDelayChooser.getSelected();

    autoChoice = 0;// m_robotContainer.m_setup.autoChooser.getSelected();

    new SetLogItemsState(m_robotContainer.m_shooter, m_robotContainer.m_tilt, m_robotContainer.m_turret, true)
        .schedule();

    LimeLight ll = m_robotContainer.m_limelight;
    RevTiltSubsystem tilt = m_robotContainer.m_tilt;
    RevTurretSubsystem turret = m_robotContainer.m_turret;
    RevShooterSubsystem shooter = m_robotContainer.m_shooter;
    RevDrivetrain drive = m_robotContainer.m_drive;
    CargoTransportSubsystem transport = m_robotContainer.m_transport;
    IntakesSubsystem intake = m_robotContainer.m_intakes;
    RawContoursV2 rcv2 = m_robotContainer.m_rCV2;
    GetTarget gt = m_robotContainer.m_gt;
    Compressor comp = m_robotContainer.m_compressor;

    switch (autoChoice) {

      case 0:// taxi start anywhere as agreed with other teams inside a tarmac facing in

        m_autonomousCommand = new Taxi(m_robotContainer.m_drive, -(FieldMap.robotLength + 1));

        break;

      case 1:// in front of power port, move back use shooter data index 1

        m_autonomousCommand = new LRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, gt, transport, comp,
            FieldMap.leftTarmacData);

        break;

      case 2://

        m_autonomousCommand = new RRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, gt, transport, comp,
            FieldMap.rightTarmacData);

        break;

      case 3://

        m_autonomousCommand = new RRetCenPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2, gt, transport, comp,
            FieldMap.rightCenTarmacData);

        break;

      case 4://

        m_autonomousCommand = new RRetPuS3();

        break;

      case 5:

        m_autonomousCommand = new RRetPuShootLow();

        break;

      case 6:

        m_autonomousCommand = new LRetPuShootLow(intake, drive, turret, tilt, FieldMap.leftTarmacData);

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

    // m_robotContainer.m_limelight.useVision = false;

    // new CalculateTargetDistance(m_robotContainer.m_limelight,
    // m_robotContainer.m_tilt, m_robotContainer.m_turret,
    // m_robotContainer.m_shooter).schedule(true);

    // new CalculateSpeedFromDistance(m_robotContainer.m_limelight,
    // m_robotContainer.m_tilt, m_robotContainer.m_turret,
    // m_robotContainer.m_shooter).schedule(true);

    new ChooseShooterSpeedSource(m_robotContainer.m_shooter, m_robotContainer.m_tilt, m_robotContainer.m_turret, 0)
        .schedule(true);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  /**
   * In teleop after cargos have been picked up, the tilt and turret need to be
   * pointed at the target so the Limelight can be used to lock them on.
   * 
   * The robot is designed so it can use the trench and this will be the default
   * turret / tilt positions afer a pickup. Thes axes will be positioned to a
   * place where the camera can pick up the target as the robot comes out from
   * under the control panel. At this point they will both be in position hold
   * mode until a target is seen and then they will move to lock on the target.
   * Target horizontal and vertical offsets can be used to shift the lock on
   * points. To lock on with vision a limelight useVision boolean must be set. A
   * Shoot when Ready command can be started at any point and will initiate
   * shooting when the shooter is at speed, the tilt and turret are lock on and
   * the robot is not moving. Shooting will prevent the joystick being used to
   * move the robot.
   * 
   * If the trench is not being used, the co driver picks either a right, left,
   * short straight or long straight shot turret from driver instructions. The
   * tilt angle will be move from its base 30 degree angle towards 0. It should
   * then pick up a target and both tilt and turret will lock on.
   * 
   * Shot speed is determined from distance calculated from target height and
   * camera angle. Speeds need to be empirically determined and put in a table to
   * be interpolated.
   * 
   */
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

}
