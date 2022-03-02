// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CargoTransport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;

public class RunLowerRoller extends CommandBase {
  /** Creates a new RunRollers. */
  private final CargoTransportSubsystem m_transport;
  private double rollerStartTime;
  private final double speed = .5;

  public RunLowerRoller(CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollerStartTime = Timer.getFPGATimestamp();
    m_transport.lowerRollerAtSpeed = false;
    SmartDashboard.putBoolean("ROLL", true);
    m_transport.haltLowerRoller = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("ROHlt", m_transport.haltLowerRoller);
   
    m_transport.runLowerRollerMotor(speed);

    if (Timer.getFPGATimestamp() > rollerStartTime + .75)
      m_transport.lowerRollerAtSpeed = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopLowerRollerMotor();
    m_transport.lowerRollerAtSpeed = false;
    m_transport.haltLowerRoller = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transport.haltLowerRoller;
  }
}
