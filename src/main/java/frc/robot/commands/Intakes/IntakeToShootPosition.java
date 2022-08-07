// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class IntakeToShootPosition extends CommandBase {

  private IntakesSubsystem m_intake;
  private CargoTransportSubsystem m_transport;
  private double activeShootStopTime;
  private boolean cargoAtShoot;
  private double cargoFullyAtShootTimer;
  private int loopctr;
  private boolean cargoFullyAtShoot;
  private boolean cargoAtShootInitially;

  /** Creates a new IntakeToShootPosition. */
  public IntakeToShootPosition(IntakesSubsystem intake, CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    loopctr = 0;

    cargoFullyAtShoot = false;

    cargoFullyAtShootTimer = 0;

    activeShootStopTime = Pref.getPref("LowRollStopTimeRed");

    if (Robot.getAllianceColorBlue())

      activeShootStopTime = Pref.getPref("LowRollStopTimeBlue");

    cargoAtShootInitially = m_transport.getCargoAtShoot();

    m_transport.simCargoAtShoot = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;

    cargoAtShoot = m_transport.getCargoAtShoot();

    if (loopctr < 10) {

      m_intake.lowerActiveArm();
    }

    if (loopctr > 30) {

      m_intake.runActiveIntakeMotor();
    }

    // low rollers run until cargo at shoot(low rollers) and short delay

    if (!cargoFullyAtShoot) {

      m_transport.intakeCargo();
    }

    else {

      m_transport.stopLowerRoller();

    }
    if (RobotBase.isSimulation() && loopctr > 100)

      m_transport.simCargoAtShoot = true;

    // cargo is at shoot (low rollers) so start the time delay

    if (cargoAtShoot && cargoFullyAtShootTimer == 0) {

      cargoFullyAtShootTimer = Timer.getFPGATimestamp();

      m_transport.resetPosition();
    }

    // cargo at shoot for duration of time delay

    cargoFullyAtShoot =

        cargoAtShoot && Timer.getFPGATimestamp() > cargoFullyAtShootTimer + activeShootStopTime;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRearIntakeMotor();
    m_intake.stopFrontIntakeMotor();
    m_intake.raiseRearArm();
    m_intake.raiseFrontArm();
    m_transport.stopLowerRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cargoAtShootInitially || cargoFullyAtShoot;
  }
}
