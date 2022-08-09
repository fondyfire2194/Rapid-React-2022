// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class MoveCargoToShootPosition extends CommandBase {
  /** Creates a new MoveCargoToShootPosition. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean cargoAtShoot;
  private boolean rearIntakeToStart;
  private boolean frontIntakeToStart;
  private double intakeStartTimer;
  private boolean cargoFullyAtShoot;
  private boolean cargoMovingToShootPosition;
  private double intakeStartDelayTime = .75;
  private double cargoFullyAtShootTimer;
  private double fullyAtShootTime;
  private boolean noCargoAtIntakesInitially;
  private int loopCtr;
  private int latchl0opctr;

  public MoveCargoToShootPosition(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
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

    cargoMovingToShootPosition = false;

    rearIntakeToStart = false;

    frontIntakeToStart = false;

    cargoFullyAtShoot = false;

    fullyAtShootTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      fullyAtShootTime = Pref.getPref("LowRollStopTimeBlue");

    m_transport.wrongCargoColor = false;

    m_shooter.wrongCargoColor = false;

    intakeStartTimer = 0;

    cargoFullyAtShootTimer = 0;

    noCargoAtIntakesInitially = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear();

    loopCtr = 0;

    latchl0opctr = 0;

    m_transport.simCargoAtShoot = false;
    m_intake.simCargoAtFrontIntake = RobotBase.isSimulation();
    // m_intake.simCargoAtRearIntake = RobotBase.isSimulation();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
    // if no cargo at shoot and cargo available at front then run front intake motor
    // after a short delay to get low roller up to speed

    cargoAtShoot = m_transport.getCargoAtShoot();

    if (!cargoAtShoot && !frontIntakeToStart && !rearIntakeToStart &&

        m_intake.getCargoAtFront()) {

      frontIntakeToStart = true;
    }

    // if no cargo at shoot and cargo available at rear then run rear intake motor

    if (!cargoAtShoot && !frontIntakeToStart && !rearIntakeToStart

        && m_intake.getCargoAtRear()) {

      rearIntakeToStart = true;
    }

    // start low rollers before intake motor

    if (frontIntakeToStart || rearIntakeToStart) {

      cargoMovingToShootPosition = true;

      if (intakeStartTimer == 0) {

        intakeStartTimer = Timer.getFPGATimestamp();
      }
    }

    if (cargoMovingToShootPosition && !cargoFullyAtShoot) {

      m_transport.intakeCargo();

    } else {

      m_transport.stopLowerRoller();
    }

    if (Timer.getFPGATimestamp() > intakeStartTimer + intakeStartDelayTime) {

      if (frontIntakeToStart) {

        m_intake.runFrontIntakeMotor();
      }

      if (rearIntakeToStart) {

        if (latchl0opctr == 0)
          latchl0opctr = loopCtr;

        if (loopCtr - latchl0opctr < 10)
          m_intake.runRearIntakeMotor();
      }
    }

    if (RobotBase.isSimulation() && loopCtr > 35)

      m_intake.simCargoAtFrontIntake = false;

    if (RobotBase.isSimulation() && loopCtr > 80)

      m_transport.simCargoAtShoot = true;

    // cargo is seen at shoot sensor - start timer to stop low roller

    if (cargoMovingToShootPosition && cargoAtShoot && cargoFullyAtShootTimer == 0) {

      cargoFullyAtShootTimer = Timer.getFPGATimestamp();

      m_transport.resetPosition();// test for how far cargo travels after seeing sensor

    }

    // second cargo now fully at shoot position so end intakink

    if (cargoMovingToShootPosition && cargoFullyAtShootTimer != 0 &&

        Timer.getFPGATimestamp() > (cargoFullyAtShootTimer + fullyAtShootTime)) {

      cargoFullyAtShoot = true;

      cargoMovingToShootPosition = false;

      // check for same color as alliance and drop shooter speed if wrong

      m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();

      if (m_transport.wrongCargoColor)

        m_shooter.wrongCargoColor = true;
    }

  }

  // no second cargo so end when released cargo is out of shooter

  // end this shoot routine when second cargo fully arrives at low rollers (shoot
  // poasition)
  // shooter keeps running to make sure cargo being fired exits correctly

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.distanceTraveledToCargoEndPosition = m_transport.getPosition();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtIntakesInitially || cargoFullyAtShoot;
  }
}
