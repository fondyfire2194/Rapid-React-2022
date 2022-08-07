// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootOneCargo extends CommandBase {
  /** Creates a new ShootOneCargo. */
  private RevShooterSubsystem m_shooter;
  private CargoTransportSubsystem m_transport;
  private IntakesSubsystem m_intake;
  private boolean cargoAtShoot;
  private boolean cargoShooting;
  private double cargoShootTimer;
  private double cargoClearShooterTime = .25;
  private boolean noCargoAtShootInitially;
  private boolean cargoClearOfShoot;
  private boolean shooterNotRunning;
  private int loopCtr;

  public ShootOneCargo(RevShooterSubsystem shooter, CargoTransportSubsystem transport,
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
    m_transport.simCargoAtShoot = RobotBase.isSimulation();
    noCargoAtShootInitially = !m_transport.getCargoAtShoot();
    cargoShootTimer = 0;
    m_shooter.isShooting = true;
    cargoShooting = false;
    cargoClearOfShoot = false;
    shooterNotRunning = !m_shooter.getShooterAtSpeed()

        || !m_shooter.getTopRollerAtSpeed();

    m_intake.simCargoAtFrontIntake = RobotBase.isSimulation();

    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopCtr++;

    cargoAtShoot = m_transport.getCargoAtShoot();

    if (m_shooter.isAtSpeed && !cargoShooting && cargoAtShoot) {

      cargoShooting = true;

    }

    if (cargoShooting) {

      m_transport.releaseCargo();// low rollers run to feed cargo to shooter

    } else {

      m_transport.stopLowerRoller();
    }

    if (loopCtr > 100) {

      m_transport.simCargoAtShoot = false;
    }

    // cargo released to shoot wheels and clear of sensor - start timer to make sure
    // it is clear of shooter wheel before bringing in second cargo

    if (cargoShooting && !cargoAtShoot && cargoShootTimer == 0) {

      cargoShootTimer = Timer.getFPGATimestamp();

    }

    cargoClearOfShoot = cargoShootTimer != 0 &&

        Timer.getFPGATimestamp() >= (cargoShootTimer + cargoClearShooterTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cargoShooting = false;
    cargoShootTimer = 0;
    cargoClearOfShoot = false;
    m_shooter.isShooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noCargoAtShootInitially || shooterNotRunning || cargoClearOfShoot;
  }
}
