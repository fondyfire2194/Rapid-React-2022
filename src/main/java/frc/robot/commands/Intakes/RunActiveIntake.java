// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private CargoTransportSubsystem m_transport;

  private boolean stopActiveIntakeNow;

  public RunActiveIntake(IntakesSubsystem intake, CargoTransportSubsystem transport) {
 
    m_intake = intake;

    m_transport = transport;

    addRequirements(m_intake);
  }

  public void initialize() {

    stopActiveIntakeNow = false;

    m_intake.setFrontCurrentLimit(20);

    m_intake.setRearCurrentLimit(20);

  }

  @Override

  public void execute() {

    m_intake.lowerActiveArm();

    if (m_transport.getCargoAtShoot()) {

      if (m_intake.useFrontIntake) {

        stopActiveIntakeNow = m_intake.getCargoAtFront();

        m_intake.setFrontCurrentLimit(10);

      }

      if (!m_intake.useFrontIntake) {

        stopActiveIntakeNow = m_intake.getCargoAtRear();

        m_intake.setRearCurrentLimit(10);
      }
    }

    if (stopActiveIntakeNow) {

      m_intake.m_frontIntakeMotor.stopMotor();

      m_intake.m_rearIntakeMotor.stopMotor();
    }

    if (!m_transport.getCargoAtShoot() || !m_intake.getCargoAtFront()) {

      if (!stopActiveIntakeNow)

        m_intake.runActiveIntakeMotor();
    }

    m_transport.intakeLowerRollerMotor();

  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_transport.stopLowerRoller();
    stopActiveIntakeNow = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
