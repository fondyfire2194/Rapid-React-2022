/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
 

  public RobotContainer m_robotContainer;
  
  public double timeToStart;
  int tst;
  private int loopCtr;

  public static double[] data = { 0, 0, 0, 0, 0 };

  public static double matchTimeRemaining;

  public static boolean hideOppCargo;
  public static final int FRONT_INTAKE_1 = 3;
  public static final int FRONT_INTAKE_2 = 2;

  public static final int REAR_INTAKE_1 = 1;
  public static final int REAR_INTAKE_2 = 0;



  public final DoubleSolenoid m_rearIntakeArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      REAR_INTAKE_1,REAR_INTAKE_2);


  public final DoubleSolenoid m_frontIntakeArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      FRONT_INTAKE_1, FRONT_INTAKE_2);

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

    m_frontIntakeArm.set(DoubleSolenoid.Value.kReverse);

    m_rearIntakeArm.set(DoubleSolenoid.Value.kReverse);

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

   
    m_robotContainer.m_drive.voltsValue = m_robotContainer.getThrottle();

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

  
    }

  

  

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

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
 

    Shuffleboard.update();

    Shuffleboard.startRecording();

    m_robotContainer.m_drive.setIdleMode(true);

    CommandScheduler.getInstance().cancelAll();

  

     
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  public void teleopPeriodic() {
    m_robotContainer.m_drive.throttleValue=m_robotContainer.getThrottle();

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
