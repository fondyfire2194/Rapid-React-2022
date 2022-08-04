// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CargoTransport;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;

public class DelayOffLowRoller extends CommandBase {
  /** Creates a new DelayOffLowRoller. */
  private CargoTransportSubsystem m_transport;

  public DelayOffLowRoller(CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transport.intakeCargo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopLowerRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transport.getCargoAtShoot();
  }
}
