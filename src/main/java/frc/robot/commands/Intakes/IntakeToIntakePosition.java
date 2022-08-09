// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class IntakeToIntakePosition extends CommandBase {

  private final IntakesSubsystem m_intake;

  private final CargoTransportSubsystem m_transport;

  private boolean stopActiveIntake;

  private double cargoFullyAtIntakeTimer;

  private double intakeStopTime = .0;

  private boolean noRoomForCargo;

  private boolean cargoFullyAtIntake;

  private int loopctr;

  public IntakeToIntakePosition(final IntakesSubsystem intake, final CargoTransportSubsystem transport) {

    m_intake = intake;

    m_transport = transport;

    addRequirements(m_intake, m_transport);
  }

  public void initialize() {

    loopctr = 0;

    cargoFullyAtIntakeTimer = 0;

    stopActiveIntake = false;

    noRoomForCargo = m_transport.getCargoAtShoot() && m_intake.getCargoAtActiveIntake();

  }

  @Override

  public void execute() {

    loopctr++;

    if (loopctr > 1) {

      m_intake.lowerActiveArm();
    }

    // run intake until first cargo is at shoot position (lower rollers)

    // intake motor runs until end of routine

    if (loopctr > 3) {

      m_intake.runActiveIntakeMotor();
    }

    // second cargo is at active intake position
    // routine will end after short time delay to make sure caro is completely in
    // robot

    if (loopctr > 100) {

      m_intake.simCargoAtFrontIntake = m_intake.useFrontIntake;

      m_intake.simCargoAtRearIntake = !m_intake.useFrontIntake;

    }

    stopActiveIntake = m_intake.getCargoAtActiveIntake();

    if (stopActiveIntake && cargoFullyAtIntakeTimer == 0) {

      cargoFullyAtIntakeTimer = Timer.getFPGATimestamp();
    }

    cargoFullyAtIntake = stopActiveIntake && cargoFullyAtIntakeTimer != 0

        && Timer.getFPGATimestamp() > cargoFullyAtIntakeTimer + intakeStopTime;

    m_intake.twoCargoOnBoard = cargoFullyAtIntake;

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

    return noRoomForCargo || cargoFullyAtIntake;
  }
}
