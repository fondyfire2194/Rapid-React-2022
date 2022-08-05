// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntake extends CommandBase {

  private final IntakesSubsystem m_intake;

  private final CargoTransportSubsystem m_transport;

  private boolean stopActiveIntake;

  private double m_startTime;

  private int loopctr;

  private double timeCargoToShoot;

  private boolean cargoAtShoot;

  private double activeShootStopTime;

  private double intakeStopTime = .05;

  public RunActiveIntake(final IntakesSubsystem intake, final CargoTransportSubsystem transport) {

    m_intake = intake;

    m_transport = transport;

    addRequirements(m_intake, m_transport);
  }

  public void initialize() {

    m_startTime = 0;

    stopActiveIntake = false;

    timeCargoToShoot = 0;

    m_transport.latchCargoAtShoot = false;

    loopctr = 0;

    activeShootStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      activeShootStopTime = Pref.getPref("LowRollStopTimeBlue");

  }

  @Override

  public void execute() {

    loopctr++;

    cargoAtShoot = m_transport.getCargoAtShoot();

    if (loopctr < 10) {

      m_intake.lowerActiveArm();
    }

    // run intake until first cargo is at shoot position (lower rollers)

    // intake motor runs until end of routine

    if (loopctr > 30) {

      m_intake.runActiveIntakeMotor();
    }

    // low rollers run until cargo at shoot(low rollers) and short delay

    if (!m_transport.latchCargoAtShoot) {

      m_transport.intakeCargo();
    }

    else {

      m_transport.stopLowerRoller();

    }

    // cargo is at shoot (low rollers) so start the time delay

    if (cargoAtShoot && timeCargoToShoot == 0) {

      timeCargoToShoot = Timer.getFPGATimestamp();
    }

    // cargo at shoot for duration of time delay

    m_transport.latchCargoAtShoot = m_transport.latchCargoAtShoot

        || cargoAtShoot && Timer.getFPGATimestamp() > timeCargoToShoot + activeShootStopTime;

    // second cargo is at active intake position
    // routine will end after short time delay to make sure caro is completely in
    // robot

    stopActiveIntake = cargoAtShoot && m_intake.getCargoAtActiveIntake();

    if (stopActiveIntake && m_startTime != 0) {

      m_startTime = Timer.getFPGATimestamp();
    }

    m_intake.twoCargoOnBoard = true;

  }

  @Override
  public void end(final boolean interrupted) {
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_intake.raiseRearArm();
    m_intake.raiseFrontArm();
    stopActiveIntake = false;
    m_transport.stopLowerRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_startTime != 0 && Timer.getFPGATimestamp() > m_startTime + intakeStopTime;
  }
}
