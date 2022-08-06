// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class AltShootCargo extends CommandBase {

  /** Creates a new AltShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean frontIntakeToStart;
  private boolean rearIntakeToStart;

  private boolean cargoAtShoot;
  private boolean noCargoAtStart;

  private double timeCargoToShoot;

  private boolean secondCargoAtShoot;
  private boolean noMoreCargo;
  private double activeShootStopTime;
  private double cargoReleaseTime = .25;
  private boolean cargoReleasing;
  private double cargoReleaseTimer;
  private double intakeStartTimer;
  private boolean cargoIntaking;

  public AltShootCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
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

    frontIntakeToStart = false;

    rearIntakeToStart = false;

    m_intake.cargoAtBothIntakes = m_intake.getCargoAtFront() && m_intake.getCargoAtRear();

    noCargoAtStart = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear() && !m_transport.getCargoAtShoot();

    timeCargoToShoot = 0;

    cargoReleaseTimer = 0;

    cargoReleasing = false;

    cargoIntaking = false;

    noMoreCargo = false;

    secondCargoAtShoot = false;

    activeShootStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      activeShootStopTime = Pref.getPref("LowRollStopTimeBlue");

    m_transport.wrongCargoColor = false;

    m_shooter.wrongCargoColor = false;

    intakeStartTimer = 0;

    m_transport.stopLowerRoller();

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

    // make sure the loww rollers have a command

    if (!cargoReleasing && !cargoIntaking) {

      m_transport.stopLowerRoller();
    }

    // if intakes aren't delivering second cargo and cargo is available at shoot
    // position sensor, shoot by starting low rollers

    if (cargoReleasing || (!frontIntakeToStart && !rearIntakeToStart && cargoAtShoot && m_shooter.getShooterAtSpeed()

        && m_shooter.getTopRollerAtSpeed())) {

      m_transport.releaseCargo();// low rollers start

      cargoReleasing = true;

    }

    // cargo released to shoot wheels and clear of sensor - start timer to make sure
    // it is clear before bringing in second cargo

    if (cargoReleasing && !cargoAtShoot && cargoReleaseTimer == 0) {

      cargoReleaseTimer = Timer.getFPGATimestamp();

    }

    if (cargoReleasing && cargoReleaseTimer != 0 && Timer.getFPGATimestamp() > cargoReleaseTimer + cargoReleaseTime) {

      cargoReleasing = false;
    }

    // if no cargo at shoot and cargo available at front then run front intake motor
    // after a short delay to get low roller up to speed

    if (!cargoAtShoot && !cargoReleasing && !rearIntakeToStart && (m_intake.getCargoAtFront() || frontIntakeToStart)) {

      frontIntakeToStart = true;

      if (intakeStartTimer == 0)

        intakeStartTimer = Timer.getFPGATimestamp();

    }

    // if no cargo at shoot and cargo available at rear then run rear intake motor

    if (!cargoAtShoot && !cargoReleasing && !frontIntakeToStart && m_intake.getCargoAtRear() || rearIntakeToStart) {

      rearIntakeToStart = true;

      if (intakeStartTimer == 0)

        intakeStartTimer = Timer.getFPGATimestamp();

    }

    // second cargo on its way from an intake so run low roller until it arrives at
    // sensor then delay, stop low roller and end this routine

    if (frontIntakeToStart || rearIntakeToStart) {

      if (!secondCargoAtShoot) {

        m_transport.intakeCargo();

        cargoIntaking = true;
      }

      if (Timer.getFPGATimestamp() > intakeStartTimer + .5) {

        if (frontIntakeToStart) {

          m_intake.runFrontIntakeMotor();
        }

        if (rearIntakeToStart) {

          m_intake.runRearIntakeMotor();
        }

      }

      // second cargo at shoot sensor - start timer to stop low roller

      if (cargoIntaking && cargoAtShoot && timeCargoToShoot == 0) {

        timeCargoToShoot = Timer.getFPGATimestamp();

        m_transport.resetPosition();

      }

      // second cargo now fully at shoot position so stop low rollers

      if (cargoIntaking && timeCargoToShoot != 0 && Timer.getFPGATimestamp() > timeCargoToShoot + activeShootStopTime) {

        secondCargoAtShoot = true;

        cargoIntaking = false;

        m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

        if (m_transport.wrongCargoColor)

          m_shooter.wrongCargoColor = true;
      }
    }

    // no second cargo so end when released cargo is out of shooter

    if (!cargoAtShoot && !cargoReleasing && !cargoIntaking &&

        !frontIntakeToStart && !rearIntakeToStart) {

      noMoreCargo = true;
    }

    // end this shoot routine when second cargo arrives at low rollers (shoot poasition)
    // shooter keeps running to make sure cargo being fired exits correctly

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (noMoreCargo) {
      m_shooter.stop();
      m_shooter.stopTopRoller();
    }
    m_transport.stopLowerRoller();
    m_shooter.isShooting = false;
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_shooter.wrongCargoColor = false;
    cargoIntaking = false;
    m_transport.distanceToCargoEndPosition = m_transport.getPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || secondCargoAtShoot || noMoreCargo;
  }
}
