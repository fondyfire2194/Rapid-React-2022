// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private CargoTransportSubsystem m_transport;

  private boolean stopActiveIntakeNow;

  private double m_startTime;

  public RunActiveIntake(IntakesSubsystem intake, CargoTransportSubsystem transport) {

    m_intake = intake;

    m_transport = transport;

    addRequirements(m_intake);
  }

  public void initialize() {

    stopActiveIntakeNow = false;

  }

  @Override

  public void execute() {
    if (m_intake.useFrontIntake)

      Shuffleboard.selectTab("FrontIntakeCamera");

    else

      Shuffleboard.selectTab("RearIntakeCamera");

    m_intake.lowerActiveArm();

    // watch for second cargo and latch its arrival
    // stop intake quickly and latch cargo at intake in case it goes
    // out of sensor range

    if (m_transport.getCargoAtShoot()) {

      if (m_intake.useFrontIntake) {

        stopActiveIntakeNow = m_intake.getCargoAtFront();

        m_startTime = Timer.getFPGATimestamp();

        m_intake.twoCargoOnBoard = true;

      }

      if (!m_intake.useFrontIntake) {

        stopActiveIntakeNow = m_intake.getCargoAtRear();

        m_startTime = Timer.getFPGATimestamp();

        m_intake.twoCargoOnBoard = true;

      }
    }

    if (stopActiveIntakeNow && Timer.getFPGATimestamp() > m_startTime + .1) {

      m_intake.m_frontIntakeMotor.stopMotor();

      m_intake.m_rearIntakeMotor.stopMotor();
    }

    if (!m_transport.getCargoAtShoot() || !m_intake.getCargoAtFront()) {

      if (!stopActiveIntakeNow)

        m_intake.runActiveIntakeMotor();
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

    Shuffleboard.selectTab("Competition");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopActiveIntakeNow || m_transport.wrongCargoColor;// stopActiveIntakeNow;
  }
}
