// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CargoTransport;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;

public class RunLowerRoller extends CommandBase {
  /** Creates a new RunRollers. */
  private final CargoTransportSubsystem m_transport;
  private final double m_rpm;

  public RunLowerRoller(CargoTransportSubsystem transport, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    m_transport.runLowerRollerMotor(m_rpm);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopLowerRoller();
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transport.haltLowerRollerMotor;
  }
}
