// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private CargoTransportSubsystem m_transport;

  private boolean stopActiveIntakeNow;

  private double m_startTime;

  private int loopctr;

  public RunActiveIntake(IntakesSubsystem intake, CargoTransportSubsystem transport) {

    m_intake = intake;

    m_transport = transport;

    addRequirements(m_intake);
  }

  public void initialize() {

    stopActiveIntakeNow = false;

    loopctr = 0;

  }

  @Override

  public void execute() {

    if (loopctr < 30)

      loopctr++;

    // if (DriverStation.isTeleop() && m_intake.useFrontIntake && m_intake.useFrontCamera)

    //   Shuffleboard.selectTab("FrontIntakeCamera");

    // if (DriverStation.isTeleop() && !m_intake.useFrontIntake && m_intake.useRearCamera)

    //   Shuffleboard.selectTab("RearIntakeCamera");

    m_intake.lowerActiveArm();

    // m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

    // run intake until first cargo is at lower rollers

    if (!m_transport.getCargoAtShoot() && loopctr > 10) {

      // if (!stopActiveIntakeNow)

      m_intake.runActiveIntakeMotor();
    }

    // watch for second cargo and latch its arrival
    // stop intake quickly and latch cargo at intake in case it goes
    // out of sensor range

    if (m_transport.getCargoAtShoot() && !stopActiveIntakeNow) {

      if (m_intake.useFrontIntake) {

        m_intake.runFrontIntakeMotor();

        stopActiveIntakeNow = m_intake.getCargoAtFront();

        m_startTime = Timer.getFPGATimestamp();

        m_intake.twoCargoOnBoard = true;

      }

      if (!m_intake.useFrontIntake && !stopActiveIntakeNow) {

        m_intake.runRearIntakeMotor();

        stopActiveIntakeNow = m_intake.getCargoAtRear();

        m_startTime = Timer.getFPGATimestamp();

        m_intake.twoCargoOnBoard = true;

      }
    }
    SmartDashboard.putBoolean("SAIN", stopActiveIntakeNow);

    if (stopActiveIntakeNow && m_startTime != 0) {

      m_intake.m_frontIntakeMotor.stopMotor();

      m_intake.m_rearIntakeMotor.stopMotor();
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_intake.raiseRearArm();
    m_intake.raiseFrontArm();
    stopActiveIntakeNow = false;
    m_transport.stopLowerRoller();
    m_intake.stopLowerRoller = true;

    if (RobotBase.isReal())

      Shuffleboard.selectTab("Competition");

    else

      Shuffleboard.selectTab("Simulation");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopActiveIntakeNow && m_startTime != 0 && Timer.getFPGATimestamp() > m_startTime + .1
        || m_transport.wrongCargoColor;// stopActiveIntakeNow;
  }
}
