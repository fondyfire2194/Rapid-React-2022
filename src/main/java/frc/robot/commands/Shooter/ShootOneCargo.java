// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootOneCargo extends CommandBase {
  /** Creates a new ShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private int m_speedSource;
  private IntakesSubsystem m_intake;
  private boolean frontIntakeRunning;
  private boolean rearIntakeRunning;

  private boolean cargoAtShoot;
  private boolean noCargoAtStart;

  private boolean oneShot;

  private double m_rpm;

  public ShootOneCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
      IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    frontIntakeRunning = false;

    rearIntakeRunning = false;

    oneShot = false;

    m_intake.cargoAtBothIntakes = m_intake.getCargoAtFront() && m_intake.getCargoAtRear();

    noCargoAtStart = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear() && !m_transport.getCargoAtShoot();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    cargoAtShoot = m_transport.getCargoAtShoot();

    m_rpm = m_shooter.getSpeedSource();

    m_shooter.runShooterPlusRoller(m_rpm);

    if ((!oneShot) && cargoAtShoot && m_shooter.atSpeed() && m_shooter.getTopRollerAtSpeed()) {

      m_transport.releaseCargo();

      if (!cargoAtShoot) {

        oneShot = true;
      }
    }

    if (!cargoAtShoot && (m_intake.getCargoAtFront() || frontIntakeRunning)) {

      m_intake.runFrontIntakeMotor();

      frontIntakeRunning = true;
    }

    if (!cargoAtShoot && !frontIntakeRunning && (m_intake.getCargoAtRear() || rearIntakeRunning)) {

      m_intake.runRearIntakeMotor();

      rearIntakeRunning = true;
    }

    if (frontIntakeRunning) {

      frontIntakeRunning = !cargoAtShoot;
    }

    if (rearIntakeRunning) {

      rearIntakeRunning = !cargoAtShoot;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.useSpeedSlider = false;
    m_shooter.stop();
    m_shooter.stopTopRoller();
    m_transport.stopLowerRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || (!frontIntakeRunning && !rearIntakeRunning && oneShot);
  }
}
