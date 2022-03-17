// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CargoTransport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.CargoTransportSubsystem;

public class RunLowerRollerIntake extends CommandBase {
  /** Creates a new RunRollers. */
  private final CargoTransportSubsystem m_transport;
  private boolean latchCargoAtShoot;
  private double m_startTime;

  public RunLowerRollerIntake(CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;

    addRequirements(m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.haltLowerRollerMotor = false;
    latchCargoAtShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_transport.runLowerAtVelocity(Pref.getPref("LowRollIntakeRPM"));

    if (!latchCargoAtShoot && m_transport.getCargoAtShoot()) {
      m_startTime = Timer.getFPGATimestamp();
    }

    if (!latchCargoAtShoot)

      latchCargoAtShoot = m_transport.getCargoAtShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopLowerRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return latchCargoAtShoot && Timer.getFPGATimestamp() > m_startTime + Pref.getPref("LowRollStopTime");

  }
}
