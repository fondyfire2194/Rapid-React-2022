// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootCargo extends CommandBase {
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
  private double m_startTime;
  private boolean cargoAtFront;
  private boolean cargoAtRear;
  private boolean m_shootTwo;

  public ShootCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
      IntakesSubsystem intake, boolean shootTwo) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;
    m_shootTwo = shootTwo;

    addRequirements(m_intake, m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.isShooting = true;

    frontIntakeRunning = false;

    rearIntakeRunning = false;

    oneShot = false;

    m_intake.cargoAtBothIntakes = m_intake.getCargoAtFront() && m_intake.getCargoAtRear();

    noCargoAtStart = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear() && !m_transport.getCargoAtShoot();

    m_startTime = 0;

    m_transport.latchCargoAtShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   // m_shooter.runShooterPlusRoller(m_shooter.getRPMFromSpeedSource());

    cargoAtShoot = m_transport.getCargoAtShoot();

    // release cargo to shooter from lower roller
    // once it clears the at-shoot sensor stop the lower roller if
    // there is no cargo at either intake

    // if there is more cargo let the low roller keep running
    // if not stop the roller
    // if there is more cargo start the appropriate intake motor
    // to drive it into the low roller
    // once detected at the low rollers stop the roller and intake motor
    // command ends there

    if (!oneShot && cargoAtShoot && m_shooter.atSpeed() && m_shooter.getTopRollerAtSpeed()) {

      m_transport.releaseCargo();

      m_intake.twoCargoOnBoard = false;

      if (!cargoAtShoot) {

        cargoAtFront = m_intake.getCargoAtFront() || m_intake.cargoAtFrontIntake;

        cargoAtRear = m_intake.getCargoAtRear() || m_intake.cargoAtRearIntake;

        if (!cargoAtFront && !cargoAtRear) {

          m_transport.stopLowerRoller();

        }

        oneShot = true;

      }
    }

    if (!cargoAtShoot && (cargoAtFront || frontIntakeRunning)) {

      m_intake.runFrontIntakeMotor();

      m_intake.cargoAtFrontIntake = false;

      frontIntakeRunning = true;
    }

    if (!cargoAtShoot && !frontIntakeRunning && (cargoAtRear || rearIntakeRunning)) {

      m_intake.runRearIntakeMotor();

      m_intake.cargoAtRearIntake = false;

      rearIntakeRunning = true;
    }

    if (frontIntakeRunning || rearIntakeRunning) {

      if (!m_shootTwo && !m_transport.latchCargoAtShoot && m_transport.getCargoAtShoot()) {

        m_startTime = Timer.getFPGATimestamp();

        m_transport.latchCargoAtShoot = true;

        m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.useSpeedSlider = false;
    // m_shooter.stop();
    // m_shooter.stopTopRoller();
    m_transport.stopLowerRoller();
    m_shooter.isShooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || (m_transport.latchCargoAtShoot && m_startTime != 0

        && Timer.getFPGATimestamp() > m_startTime + Pref.getPref("LowRollStopTime"))

        || (!frontIntakeRunning && !rearIntakeRunning);
  }
}
