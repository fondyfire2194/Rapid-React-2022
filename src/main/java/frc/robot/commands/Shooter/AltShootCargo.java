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

  private double timeCargoFullyAtShoot;

  private boolean secondCargoFullyAtShoot;
  private boolean noMoreCargo;
  private double fullyAtShootTime;
  private double cargoClearShooterTime = .25;
  private double intakeStartDelayTime = .5;
  private boolean cargoShooting;
  private double cargoShootTimer;
  private double intakeStartTimer;
  private boolean cargoMovingToShootPosition;

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

    timeCargoFullyAtShoot = 0;

    cargoShootTimer = 0;

    cargoShooting = false;

    cargoMovingToShootPosition = false;

    noMoreCargo = false;

    secondCargoFullyAtShoot = false;

    fullyAtShootTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      fullyAtShootTime = Pref.getPref("LowRollStopTimeBlue");

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

    if (!cargoShooting && !cargoMovingToShootPosition) {

      m_transport.stopLowerRoller();
    }

    // check conditions for shooting

    if (!cargoMovingToShootPosition && cargoAtShoot && m_shooter.getShooterAtSpeed()

        && m_shooter.getTopRollerAtSpeed()) {

      cargoShooting = true;

    }
    if (cargoShooting)

      m_transport.releaseCargo();// low rollers start to feed cargo to shoter


    // cargo released to shoot wheels and clear of sensor - start timer to make sure
    // it is clear of shooter wheel before bringing in second cargo

    if (cargoShooting && !cargoAtShoot && cargoShootTimer == 0) {

      cargoShootTimer = Timer.getFPGATimestamp();

    }
    // check for cargo clear of shooter wheels
    if (cargoShooting && cargoShootTimer != 0 &&

        Timer.getFPGATimestamp() > (cargoShootTimer + cargoClearShooterTime)) {

      cargoShooting = false;
    }

    // if no cargo at shoot and cargo available at front then run front intake motor
    // after a short delay to get low roller up to speed

    if (!cargoAtShoot && !cargoShooting && !rearIntakeToStart &&

        m_intake.getCargoAtFront()) {

      frontIntakeToStart = true;

      if (intakeStartTimer == 0)

        intakeStartTimer = Timer.getFPGATimestamp();

    }

    // if no cargo at shoot and cargo available at rear then run rear intake motor

    if (!cargoAtShoot && !cargoShooting && !frontIntakeToStart

        && m_intake.getCargoAtRear()) {

      rearIntakeToStart = true;

      if (intakeStartTimer == 0)

        intakeStartTimer = Timer.getFPGATimestamp();

    }

   //start low rollers befor intake motor

    if (frontIntakeToStart || rearIntakeToStart) {

      if (!secondCargoFullyAtShoot) {

        m_transport.intakeCargo();

        cargoMovingToShootPosition = true;
      }

      if (Timer.getFPGATimestamp() > intakeStartTimer + intakeStartDelayTime) {

        if (frontIntakeToStart) {

          m_intake.runFrontIntakeMotor();
        }

        if (rearIntakeToStart) {

          m_intake.runRearIntakeMotor();
        }

      }

      // second cargo at shoot sensor - start timer to stop low roller

      if (cargoMovingToShootPosition && cargoAtShoot && timeCargoFullyAtShoot == 0) {

        timeCargoFullyAtShoot = Timer.getFPGATimestamp();

        m_transport.resetPosition();//test for how far cargo travels after seeing sensor

      }

      // second cargo now fully at shoot position so end intakink

      if (cargoMovingToShootPosition && timeCargoFullyAtShoot != 0 &&

          Timer.getFPGATimestamp() > timeCargoFullyAtShoot + fullyAtShootTime) {

        secondCargoFullyAtShoot = true;

        cargoMovingToShootPosition = false;

        //check for same color as alliance and drop shooter speed if wrong

        m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

        if (m_transport.wrongCargoColor)

          m_shooter.wrongCargoColor = true;
      }
    }

    // no second cargo so end when released cargo is out of shooter

    noMoreCargo = !cargoAtShoot && !cargoShooting && !cargoMovingToShootPosition;

    // end this shoot routine when second cargo fully arrives at low rollers (shoot
    // poasition)
    // shooter keeps running to make sure cargo being fired exits correctly

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_transport.stopLowerRoller();
    m_shooter.isShooting = false;
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_shooter.wrongCargoColor = false;
    cargoMovingToShootPosition = false;
    m_transport.distanceTraveledToCargoEndPosition = m_transport.getPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || secondCargoFullyAtShoot || noMoreCargo;
  }
}
