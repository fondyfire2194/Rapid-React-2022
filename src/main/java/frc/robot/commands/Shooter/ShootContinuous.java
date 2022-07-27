// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootContinuous extends CommandBase {

  /** Creates a new AltShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean frontIntakeRunning;
  private boolean rearIntakeRunning;

  private boolean cargoAtShoot;
  private boolean noCargoAtStart;

  private boolean noMoreCargo;

  private double cargoReleaseTime = .25;
  private boolean cargoReleasingToShootWheels;
  private double cargoReleaseTimer;
  

  public ShootContinuous(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
      IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;
    
    addRequirements(m_intake, m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooter.isShooting = true;

    frontIntakeRunning = false;

    rearIntakeRunning = false;

    m_intake.cargoAtBothIntakes = m_intake.getCargoAtFront() && m_intake.getCargoAtRear();

    noCargoAtStart = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear() && !m_transport.getCargoAtShoot();

    cargoReleaseTimer = 0;

    cargoReleasingToShootWheels = false;

    m_transport.latchCargoAtShoot = false;

    noMoreCargo = false;

    m_transport.wrongCargoColor = false;

    m_shooter.wrongCargoColor = false;

  }

  // Called every time the scheduler runs while the command is scheduled.

  // on start of command, if no cargo at shoot sensor or at either intake, routine
  // ends immediatly

  // routine should normally start with cargo at the low rollers (at shoot sensor)

  // it will release the cargo to the shooter wheels and start a cargo releasing
  // delay

  // Once the releasing delay ends, if neither intake has cargo, the routine ends

  // If an intake has cargo, it will start that intake and start the low rollers

  // The low rollers don't stop between shots but the intake delays slightly
  // before starting
  //
  @Override
  public void execute() {

    cargoAtShoot = m_transport.getCargoAtShoot();

    // if intakes aren't delivering second cargo and cargo is available at shoot
    // position sensor, shoot by starting low rollers

    if (cargoAtShoot && m_shooter.getShooterAtSpeed() && m_shooter.getTopRollerAtSpeed() || cargoReleasingToShootWheels) {

      m_transport.releaseCargo();// low rollers run

      cargoReleasingToShootWheels = true;

      if (!cargoAtShoot && cargoReleaseTimer == 0) {

        cargoReleaseTimer = Timer.getFPGATimestamp();

      }
    }

    if (cargoReleasingToShootWheels && Timer.getFPGATimestamp() > cargoReleaseTimer + cargoReleaseTime) {

      cargoReleasingToShootWheels = false;
    }

    // if no cargo at shoot and cargo available at front then run front intake motor

    if (!cargoAtShoot && !cargoReleasingToShootWheels && !rearIntakeRunning && (m_intake.getCargoAtFront() || frontIntakeRunning)) {

      m_intake.runFrontIntakeMotor();

      frontIntakeRunning = true;
    }

    // if no cargo at shoot and cargo available at rear then run rear intake motor

    if (!cargoAtShoot && !cargoReleasingToShootWheels && !frontIntakeRunning && (m_intake.getCargoAtRear() || rearIntakeRunning)) {

      m_intake.runRearIntakeMotor();

      rearIntakeRunning = true;
    }

    // second cargo on its way from an intake so run low roller until it arrives at
    // sensor then delay, stop low roller and end this routine

    if (frontIntakeRunning || rearIntakeRunning) {

      m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

      if (m_transport.wrongCargoColor)

        m_shooter.wrongCargoColor = true;
    }

    // no second cargo so end when released cargo is out of shooter

    if (!cargoAtShoot && !cargoReleasingToShootWheels ) {

      noMoreCargo = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.stop();

    m_shooter.stopTopRoller();

    m_transport.stopLowerRoller();

    m_intake.stopFrontIntakeMotor();

    m_intake.stopRearIntakeMotor();

    m_shooter.isShooting = false;

    m_shooter.wrongCargoColor = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || noMoreCargo

    ;
  }
}
