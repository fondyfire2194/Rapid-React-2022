// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class AltShootCargo extends CommandBase {

  /** Creates a new AltShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean frontIntakeRunning;
  private boolean rearIntakeRunning;

  private boolean cargoAtShoot;
  private boolean noCargoAtStart;

  private double timeCargoToLowRoller;

  private boolean secondCargoAtLowRoller;
  private boolean noMoreCargo;
  private double activeLowStopTime;
  private double cargoReleaseTime = .25;
  private boolean cargoReleasing;
  private double cargoReleaseTimer;
  private LimeLight m_ll;

  public AltShootCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
      IntakesSubsystem intake, LimeLight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_intake = intake;
    m_ll = ll;

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

    timeCargoToLowRoller = 0;

    cargoReleaseTimer = 0;

    cargoReleasing = false;

    m_transport.latchCargoAtShoot = false;

    noMoreCargo = false;

    secondCargoAtLowRoller = false;

    activeLowStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      activeLowStopTime = Pref.getPref("LowRollStopTimeBlue");

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

  // when the low rollers see the cargo they will start a delay time and stop
  // after it expires. Routine then ends

  //
  @Override
  public void execute() {

    cargoAtShoot = m_transport.getCargoAtShoot();

    // if intakes aren't delivering second cargo and cargo is available at shoot
    // position sensor, shoot by starting low rollers

    if (cargoReleasing || (!frontIntakeRunning && !rearIntakeRunning && cargoAtShoot && m_shooter.getShooterAtSpeed()

        && m_shooter.getTopRollerAtSpeed())) {

      m_transport.releaseCargo();// low rollers start

      cargoReleasing = true;

      if (!cargoAtShoot && cargoReleaseTimer == 0) {

        cargoReleaseTimer = Timer.getFPGATimestamp();

      }
    }

    if (cargoReleasing && Timer.getFPGATimestamp() > cargoReleaseTimer + cargoReleaseTime) {

      cargoReleasing = false;
    }

    // if no cargo at shoot and cargo available at front then run front intake motor

    if (!cargoAtShoot && !cargoReleasing && !rearIntakeRunning && (m_intake.getCargoAtFront() || frontIntakeRunning)) {

      m_intake.runFrontIntakeMotor();

      frontIntakeRunning = true;
    }

    // if no cargo at shoot and cargo available at rear then run rear intake motor

    if (!cargoAtShoot && !cargoReleasing && !frontIntakeRunning && (m_intake.getCargoAtRear() || rearIntakeRunning)) {

      m_intake.runRearIntakeMotor();

      rearIntakeRunning = true;
    }

    // second cargo on its way from an intake so run low roller until it arrives at
    // sensor then delay, stop low roller and end this routine

    if (frontIntakeRunning || rearIntakeRunning) {

      if (!secondCargoAtLowRoller)

        m_transport.intakeCargo();

      if (timeCargoToLowRoller == 0 && cargoAtShoot) {

        timeCargoToLowRoller = Timer.getFPGATimestamp();

        m_transport.latchCargoAtShoot = true;

        m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

        m_shooter.wrongCargoColor = m_transport.wrongCargoColor;

      }

    }
    // no second cargo so end when released cargo is out of shooter

    if (!cargoAtShoot && !cargoReleasing && !frontIntakeRunning && !rearIntakeRunning) {

      noMoreCargo = true;
    }

    // end this shoot routine when second cargo arrives at low rollers
    // shooter keeps running to make sure cargo being fired exits correctly

    secondCargoAtLowRoller = m_transport.latchCargoAtShoot

        && Timer.getFPGATimestamp() > timeCargoToLowRoller + activeLowStopTime;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (noMoreCargo) {
      m_shooter.stop();
      m_shooter.stopTopRoller();
      // m_ll.setPipeline(PipelinesConstants.ledsOffPipeline);
      // m_ll.useVision = false;
    }
    m_transport.stopLowerRoller();
    m_shooter.isShooting = false;
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || secondCargoAtLowRoller || noMoreCargo

        || m_transport.latchCargoAtShoot;
  }
}
