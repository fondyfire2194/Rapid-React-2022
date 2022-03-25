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

public class ShootCargo extends CommandBase {

  /** Creates a new ShootCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean frontIntakeRunning;
  private boolean rearIntakeRunning;

  private boolean cargoAtShoot;
  private boolean noCargoAtStart;

  private double m_startTime;
  private double m_startTime_2;

  private boolean endit1;
  private boolean endit2;
  private double activeLowStopTime;

  private boolean cargoReleasing;

  public ShootCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
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

    cargoReleasing = false;

    m_intake.cargoAtBothIntakes = m_intake.getCargoAtFront() && m_intake.getCargoAtRear();

    noCargoAtStart = !m_intake.getCargoAtFront() && !m_intake.getCargoAtRear() && !m_transport.getCargoAtShoot();

    m_startTime = 0;

    m_startTime_2 = 0;

    m_transport.latchCargoAtShoot = false;

    activeLowStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      activeLowStopTime = Pref.getPref("LowRollStopTimeBlue");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    cargoAtShoot = m_transport.getCargoAtShoot();
    

    if ((cargoAtShoot && m_shooter.atSpeed() && m_shooter.getTopRollerAtSpeed())) {

      m_transport.releaseCargo();

      
    }

    if (!cargoAtShoot && !rearIntakeRunning && (m_intake.getCargoAtFront() || frontIntakeRunning)) {

      m_intake.runFrontIntakeMotor();

      frontIntakeRunning = true;
    }

    if (!cargoAtShoot && !frontIntakeRunning && (m_intake.getCargoAtRear() || rearIntakeRunning)) {

      m_intake.runRearIntakeMotor();

      rearIntakeRunning = true;
    }
    // no second cargo

    if (!cargoAtShoot && !frontIntakeRunning && !rearIntakeRunning) {
   
      if (m_startTime == 0) {
  
        m_startTime = Timer.getFPGATimestamp();
      }

    }

    // no second cargo end

    endit1 = m_startTime != 0 && Timer.getFPGATimestamp() > m_startTime_2 + 1;

    // new cargo on its way and low roller is running

    if (frontIntakeRunning || rearIntakeRunning) {

      if (!m_transport.latchCargoAtShoot)

        m_transport.runLowerAtVelocity(Pref.getPref("LowRollIntakeRPM"));

      if (!m_transport.latchCargoAtShoot && m_transport.getCargoAtShoot()) {

        m_startTime_2 = Timer.getFPGATimestamp();

        m_transport.latchCargoAtShoot = true;

        m_transport.wrongCargoColor = m_transport.getCargoAllianceMisMatch();
      }
      // second cargo end
      
      endit2 = m_transport.latchCargoAtShoot && m_startTime != 0

          && Timer.getFPGATimestamp() > m_startTime + activeLowStopTime;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.stopTopRoller();
    m_transport.stopLowerRoller();
    m_shooter.isShooting = false;
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    cargoReleasing = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return noCargoAtStart || m_transport.wrongCargoColor || endit1 || endit2 || m_transport.latchCargoAtShoot;
  }
}
