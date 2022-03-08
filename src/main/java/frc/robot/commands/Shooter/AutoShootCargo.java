// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class AutoShootCargo extends CommandBase {
  /** Creates a new ShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private double m_startTime;
  private boolean frontIntakeStarted;
  private boolean rearIntakeStarted;

  public AutoShootCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport, IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_startTime = Timer.getFPGATimestamp();
    frontIntakeStarted = false;
    rearIntakeStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.runShooter(m_shooter.cameraCalculatedSpeed);

    if (m_shooter.atSpeed() && m_shooter.getTopRollerAtSpeed()) {

      m_transport.releaseCargo();
    }

    if (!m_transport.getCargoAtShoot() && (m_intake.getCargoAtFront() || frontIntakeStarted)) {
      frontIntakeStarted = true;
      m_startTime = Timer.getFPGATimestamp();
      m_intake.runFrontIntakeMotor();
    }

    if (!m_transport.getCargoAtShoot() && (m_intake.getCargoAtRear() || rearIntakeStarted)) {
      rearIntakeStarted = true;
      m_startTime = Timer.getFPGATimestamp();
      m_intake.runRearIntakeMotor();
      m_shooter.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_transport.stopLowerRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (!frontIntakeStarted && !rearIntakeStarted) && (Timer.getFPGATimestamp() - m_startTime) > 1

        || (frontIntakeStarted || rearIntakeStarted) && Timer.getFPGATimestamp() - m_startTime > 2;
  }
}
