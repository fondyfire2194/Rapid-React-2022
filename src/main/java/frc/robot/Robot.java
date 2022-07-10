/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Constants.ShooterRangeConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.AutoCommands.CenterHideOppCargo;
import frc.robot.commands.AutoCommands.DoNothing;
import frc.robot.commands.AutoCommands.LeftHideOppCargo;
import frc.robot.commands.AutoCommands.RetPuShootCamera;
import frc.robot.commands.AutoCommands.RetPuShootCameraTraj;
import frc.robot.commands.AutoCommands.RunCenterThirdCargo;
import frc.robot.commands.AutoCommands.RunRightFirstPickup;
import frc.robot.commands.AutoCommands.RunRightThreeCargo;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.SetRobotPose;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

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

  public static double[] data = { 0, 0, 0, 0, 0 };

  public static double matchTimeRemaining;

  public static boolean hideOppCargo;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    setNetworkTablesFlushEnabled(true);

    // Starts recording to data log

    if (RobotBase.isReal())
      DataLogManager.start();

    m_robotContainer = new RobotContainer();

    getAllianceColorBlue();

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

    m_robotContainer.m_shooter.driverThrottleValue = m_robotContainer.getThrottle();

    loopCtr++;

    SmartDashboard.putNumber("LPCTRA", loopCtr);

  }

  /**
   * .0
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

    m_robotContainer.m_limelight.setPipeline(8);

  }

  @Override
  public void disabledPeriodic() {

  }

  public void autonomousInit() {

    m_robotContainer.m_drive.resetAll();
    autoHasRun = false;
    m_robotContainer.m_limelight.allianceIsBlue = DriverStation.getAlliance() == Alliance.Blue;

    m_robotContainer.m_drive.setIdleMode(true);

    Shuffleboard.selectTab("Competition");

    Shuffleboard.startRecording();
    // get delay time

    m_startDelay = m_robotContainer.m_preOi.startDelayChooser.getSelected();

    autoChoice = m_robotContainer.m_preOi.autoChooser.getSelected();

    hideOppCargo = m_robotContainer.m_preOi.hideCargoChooser.getSelected();

    LimeLight ll = m_robotContainer.m_limelight;
    RevTiltSubsystem tilt = m_robotContainer.m_tilt;
    RevTurretSubsystem turret = m_robotContainer.m_turret;
    RevShooterSubsystem shooter = m_robotContainer.m_shooter;
    RevDrivetrain drive = m_robotContainer.m_drive;
    CargoTransportSubsystem transport = m_robotContainer.m_transport;
    IntakesSubsystem intake = m_robotContainer.m_intake;
    Compressor comp = m_robotContainer.m_compressor;
    FondyFireTrajectory fftraj = m_robotContainer.m_trajectory;

    ll.setLEDMode(LedMode.kpipeLine);
    ll.setPipeline(PipelinesConstants.noZoom960720);

    switch (autoChoice) {

      case 0:
        m_autonomousCommand = new MessageCommand("Did Nothing Auto");
        ll.setPipeline(PipelinesConstants.ledsOffPipeline);
        break;

      case 1:// taxi start anywhere as agreed with other teams inside a tarmac facing in

        m_autonomousCommand = new SequentialCommandGroup(
            new TiltMoveToReverseLimit(m_robotContainer.m_tilt),
            new ResetEncoders(m_robotContainer.m_drive),
            new ResetGyro(m_robotContainer.m_drive),
            new PositionStraight(m_robotContainer.m_drive, -1.5, .3));

        break;

      case 2:// left tarmac start

        data = FieldMap.leftTarmacData;

        // camera shoot numbers are from front bumper to hub fender which is 18" from
        // hub center
        // robot start has front bumper the robot length inside the tarmac line
        // robot is 37.75" long and tarmac line is 82 inches from fender
        // after retract will be 80 + 1.29
        // so shot length s (82 - 37.75) + 1.29 *39.37 = 44.25 + 52 = 96" = 8 ft
        // this is the start of the tilt range 2 = 11 degrees and 2300 rpm

        data[0] = -1.6;// retract point

        data[2] = ShooterRangeConstants.tiltRange3;// tilt 14 deg

        data[3] = 0;// turret will be locked to Limelight

        data[4] = 2700;

        m_autonomousCommand = new SequentialCommandGroup(

            new SetRobotPose(m_robotContainer.m_drive, fftraj.leftAutoStartRev),

            new RetPuShootCameraTraj(intake, drive, fftraj, fftraj.leftPickupRev, transport, shooter, tilt, turret, ll,
                comp, data),

            new ConditionalCommand(new LeftHideOppCargo(intake, drive, transport, shooter), new DoNothing(),
                () -> hideOppCargo));

        break;

      case 3://

        data = FieldMap.centerTarmacData;

        data[0] = -1.4;// retract point

        data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

        data[3] = 0;// turret will be locked to Limelight

        data[4] = 2700;

        m_autonomousCommand = new SequentialCommandGroup(

            new SetRobotPose(m_robotContainer.m_drive,
                fftraj.centerAutoStartRev),

            new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

            new RetPuShootCameraTraj(intake, drive, fftraj, fftraj.centerFirstPickUpRev, transport, shooter, tilt,
                turret, ll,
                comp, data),

            new ConditionalCommand(new CenterHideOppCargo(intake, drive, transport, shooter), new DoNothing(),
                () -> hideOppCargo));

        break;

      case 4:// Pick up and shoot cargo in center of field plus third cargo

        data = FieldMap.centerTarmacData;

        data[0] = -1.4;// retract point

        data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

        data[3] = 0;// turret will be locked to Limelight

        data[4] = 2700;

        m_autonomousCommand = new SequentialCommandGroup(

            new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

            new SetRobotPose(m_robotContainer.m_drive,
                fftraj.centerAutoStartRev),

            new RetPuShootCameraTraj(intake, drive, fftraj, fftraj.centerFirstPickUpRev, transport, shooter, tilt,
                turret, ll,
                comp, data),

            new RunCenterThirdCargo(drive, fftraj, intake, shooter, tilt, turret, transport, ll));

        break;

      case 5:// Pick up and shoot cargo in center of field plus third cargo

        data = FieldMap.rightTarmacData;

        data[0] = -1.4;// retract point

        data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

        data[3] = 0;// turret will be locked to Limelight

        data[4] = 2700;

        m_autonomousCommand = new SequentialCommandGroup(

            new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

            new SetRobotPose(m_robotContainer.m_drive,
                fftraj.rightCargoAutoStart),

            new RunRightFirstPickup(drive, fftraj, intake, shooter, tilt, turret, transport, ll),

            new RunRightThreeCargo(drive, fftraj, intake, shooter, tilt, turret, transport, ll),

            new RunCenterThirdCargo(drive, fftraj, intake, shooter, tilt, turret, transport, ll));

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

    CommandScheduler.getInstance().cancelAll();

    if (!m_robotContainer.m_tilt.positionResetDone)

      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule();

    m_robotContainer.m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);

    m_robotContainer.m_limelight.useVision = false;

    m_robotContainer.m_intake.setFrontActive();

    m_robotContainer.m_shooter.shootValuesSource = m_robotContainer.m_shooter.fromPreset;

    m_robotContainer.m_shooter.shootLocation = 1;
    m_robotContainer.m_shooter.presetLocationName = FieldMap.shootLocationName[m_robotContainer.m_shooter.shootLocation];
    m_robotContainer.m_shooter.shootModeName = FieldMap.shootModeName[m_robotContainer.m_shooter.shootValuesSource];

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  public void teleopPeriodic() {
    // SmartDashboard.putData("ShootSequence", m_robotContainer.ssdisp);
    // if (m_robotContainer.m_transport.wrongCargoColor) {

    // new EmptyCargoFromShooter(m_robotContainer.m_shooter,
    // m_robotContainer.m_transport)

    // .execute();
    // }

    matchTimeRemaining = DriverStation.getMatchTime();
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
