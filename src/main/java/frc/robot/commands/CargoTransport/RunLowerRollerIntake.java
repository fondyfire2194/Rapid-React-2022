// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CargoTransport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunLowerRollerIntake extends CommandBase {
  /** Creates a new RunRollers. */
  private final CargoTransportSubsystem m_transport;
  private final IntakesSubsystem m_intake;

  private double m_startTime;

  private double activeLowStopTime;

  public RunLowerRollerIntake(CargoTransportSubsystem transport, IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_intake = intake;
    addRequirements(m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.haltLowerRollerMotor = false;
    m_transport.latchCargoAtShoot = false;
    m_startTime = 0;
    m_intake.stopLowerRoller = false;

    activeLowStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())
    
      activeLowStopTime = Pref.getPref("LowRollStopTimeBlue");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_transport.runLowerAtVelocity(Pref.getPref("LowRollIntakeRPM"));

    if (!m_transport.latchCargoAtShoot && m_transport.getCargoAtShoot()) {

      m_startTime = Timer.getFPGATimestamp();

      m_transport.latchCargoAtShoot = true;

      m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();
    }

    SmartDashboard.putNumber("ISTA", m_startTime);

    SmartDashboard.putBoolean("LATCH", m_transport.latchCargoAtShoot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopLowerRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_intake.stopLowerRoller
        || m_transport.latchCargoAtShoot && m_startTime != 0
            && Timer.getFPGATimestamp() > m_startTime + activeLowStopTime;

  }
}
