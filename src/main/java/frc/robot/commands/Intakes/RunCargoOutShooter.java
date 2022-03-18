// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class RunCargoOutShooter extends CommandBase {
  /** Creates a new RunCargoOutShooter. */
  private RevShooterSubsystem m_shooter;
  private IntakesSubsystem m_intake;
  private CargoTransportSubsystem m_transport;

  public RunCargoOutShooter(RevShooterSubsystem shooter, IntakesSubsystem intake, CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_intake = intake;
    m_transport = transport;
    addRequirements(m_intake, m_transport, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.spinAtRPM(1200);
    m_shooter.runTopAtVelocity(800);
    m_transport.runLowerAtVelocity(800);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.stopTopRoller();
    m_intake.stopRearIntakeMotor();
    m_intake.stopFrontIntakeMotor();
    m_transport.stopLowerRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
