/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Constants.ShooterRangeConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.AutoCommands.AltRetPuAdvShoot;
import frc.robot.commands.AutoCommands.RetPuShootCamera;
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
  // public BooleanLogEntry rrBooleanLog;
  // public static DoubleLogEntry rrDoubleLog;
  // public static StringLogEntry rrStringLog;

  public static double[] data = { 0, 0, 0, 0, 0 };

  public static double matchTimeRemaining;

  private Pose2d startingPose;

  private boolean useLacrosse = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

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

    // m_robotContainer.m_limelight.periodic();

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

    // new CalculateTargetDistance(m_robotContainer.m_limelight,
    // m_robotContainer.m_tilt,
    // m_robotContainer.m_turret, m_robotContainer.m_shooter).schedule();
    // new SelectSpeedAndTiltByDistance(m_robotContainer.m_shooter,
    // m_robotContainer.m_tilt).schedule();

    if (RobotBase.isReal())

      Shuffleboard.selectTab("Competition");

    else

      Shuffleboard.selectTab("Simulation");

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
    Compressor comp = m_robotContainer.m_compressor;

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

      case 2:// left tarmac upper shoot

        data = FieldMap.leftTarmacData;

        startingPose = new Pose2d(6.34, 4.92, Rotation2d.fromDegrees(-42));

        if (useLacrosse) {

          data[0] = Pref.getPref("autLRtctPt");// retract point -1.29

          data[1] = Pref.getPref("autLShootPt");// shoot point -.9

          data[2] = Pref.getPref("autLTilt");// tilt 15

          data[3] = Pref.getPref("autLTu");// turret 0

          data[4] = Pref.getPref("autLRPM");// rpm 2700

          m_autonomousCommand = new SequentialCommandGroup(

              new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              new SetRobotPose(m_robotContainer.m_drive, startingPose),

              new AltRetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll,
                  comp, data));

        } else {

          // camera shoot numbers are from front bumper to hub fender which is 18" from
          // hub center
          // robot start has front bumper the robot length inside the tarmac line
          // robot is 37.75" long and tarmac line is 82 inches from fender
          // after retract will be 80 + 1.29
          // so shot length s (82 - 37.75) + 1.29 *39.37 = 44.25 + 52 = 96" = 8 ft
          // this is the start of the tilt range 2 = 11 degrees and 2300 rpm

          data[0] = -1.29;// retract point = 52"

          // data[1] return to shoot not used

          data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

          // data[3] not used// turret will be locked to Limelight

          data[4] = shooter.rpmFromCameraDistance[8 + 1];// rpm

          m_autonomousCommand = new SequentialCommandGroup(

              new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              new RetPuShootCamera(intake, drive, transport, shooter, tilt, turret, ll,
                  comp, data));

        }
        break;

      case 3:// Pick up and shoot cargo nearest field wall

        data = FieldMap.rightTarmacData;

         startingPose = new Pose2d(7.65, 2.03, Rotation2d.fromDegrees(90));

        if (useLacrosse) {

          data[0] = (Pref.getPref("autRRtctPt"));// CHANGE retract point to 1.1 meters

          data[1] = Pref.getPref("autRShootPt");// shoot point .9 meters

          data[2] = Pref.getPref("autRTilt");// tilt 5 degrees change to 15

          data[3] = Pref.getPref("autRTu");// turret +10 degrees

          data[4] = Pref.getPref("autRRPM");// rpm 3500 chamge to 2700

          m_autonomousCommand = new SequentialCommandGroup(new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              new SetRobotPose(m_robotContainer.m_drive, startingPose),

              new AltRetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll, comp,

                  data));

        }

        else {

          data[0] = -1.1;// retract point = 44" WATCH FOR WALL!

          // data[1] return to shoot not used

          data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

          data[3] = 0;// turret will be locked to Limelight

          data[4] = shooter.rpmFromCameraDistance[8 + 1];// rpm

          m_autonomousCommand = new SequentialCommandGroup(

              new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              new RetPuShootCamera(intake, drive, transport, shooter, tilt, turret, ll,
                  comp, data));
          
        }

        break;

      case 4:// Pick up and shoot cargo in right center of field

        data = FieldMap.centerTarmacData;

         startingPose = new Pose2d(6.78, 3, Rotation2d.fromDegrees(34.32));

        if (useLacrosse) {

          data[0] = Pref.getPref("autCRtctPt");// retract point -1.29

          data[1] = Pref.getPref("autCShootPt");// shoot point .9

          data[2] = Pref.getPref("autCTilt");// tilt 15

          data[3] = Pref.getPref("autCTu");// turret //change to -5

          data[4] = Pref.getPref("autCRPM");// rpm 2700

          m_autonomousCommand = new SequentialCommandGroup(new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              // new SetRobotPose(m_robotContainer.m_drive, startingPose),

              new AltRetPuAdvShoot(intake, drive, transport, shooter, tilt, turret, ll, comp,

                  data));

          // new CenterHideOppCargo(intake, drive, transport, shooter, tilt, turret, ll));
        }

        else {

          data[0] = -1.29;// retract point = 44" WATCH FOR WALL!

          // data[1] return to shoot not used

          data[2] = ShooterRangeConstants.tiltRange2;// tilt 11 deg

          data[3] = 0;// turret will be locked to Limelight

          data[4] = shooter.rpmFromCameraDistance[8 + 1];// rpm

          m_autonomousCommand = new SequentialCommandGroup(

              new TiltMoveToReverseLimit(m_robotContainer.m_tilt),

              new RetPuShootCamera(intake, drive, transport, shooter, tilt, turret, ll,
                  comp, data));


        }

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

      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(false);

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
