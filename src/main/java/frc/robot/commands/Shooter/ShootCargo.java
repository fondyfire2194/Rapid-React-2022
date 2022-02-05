/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class ShootCargo extends CommandBase {
  /**
   * Creates a new ShootCargos
   * 
   */
  private final RevShooterSubsystem m_shooter;
  private final RevTiltSubsystem m_tilt;
  private final RevTurretSubsystem m_turret;
  private final CargoTransportSubsystem m_transport;
  private final RevDrivetrain m_drive;
  private final Compressor m_compressor;
  private final LimeLight m_limelight;
  private double m_time;
  private double shotTime = .7;
  private double shotStartTime;

  private boolean okToShoot;

  private boolean getNextCargo;

  private boolean cargoReleased;
  private double cargoReleasedStartTime;

  private boolean inAuto;
  private boolean useSensors = false;
  public boolean robotStoppedFor1Sec;

  public ShootCargo(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight limelight,
      CargoTransportSubsystem transport, RevDrivetrain drive, Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;
    m_limelight = limelight;
    m_drive = drive;
    m_tilt = tilt;
    m_turret = turret;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shootCargosRunning = 1.;
    m_shooter.logShooterItems = true;
    m_shooter.shootTime = m_time;
    m_compressor.disable();
    m_shooter.isShooting = false;
    m_transport.cargoAvailable = false;

    m_transport.cargosShot = 0;
    shotStartTime = 0;

    m_limelight.setLEDMode(LedMode.kpipeLine);

    m_limelight.useVision = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotStoppedFor1Sec = m_drive.robotStoppedForOneSecond;

    inAuto = DriverStation.isAutonomous();

    okToShoot = (m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance)
        && m_limelight.getHorOnTarget(m_turret.turretVisionTolerance)) || m_shooter.useDriverSpeed;

    if (m_shooter.atSpeed() && m_transport.rollersAtSpeed && okToShoot && robotStoppedFor1Sec || m_shooter.isShooting) {

      m_shooter.isShooting = true;

    }

    if (m_shooter.isShooting && m_transport.cargoAvailable && !m_shooter.shotInProgress) {
      shotStartTime = Timer.getFPGATimestamp();
      m_shooter.shotInProgress = true;
      m_transport.cargosShot++;
      m_transport.cargoAvailable = false;

    }

    if (m_shooter.shotInProgress && Timer.getFPGATimestamp() > shotStartTime + shotTime) {

      m_shooter.shotInProgress = false;

    }

    m_shooter.okToShoot = m_shooter.isShooting && (inAuto || !m_shooter.shootOne);

    getNextCargo = m_shooter.okToShoot && !m_shooter.shotInProgress && !m_transport.cargoAvailable
        && m_transport.rollersAtSpeed && m_transport.getBallAtShoot() && m_shooter.atSpeed();

    if (getNextCargo || cargoReleased) {
      releaseOneCargo();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.cargoAvailable=false;
  
    
   // m_compressor.enableAnalog(minPressure, maxPressure);
    m_shooter.shotInProgress = false;

    m_shooter.isShooting = false;
    m_shooter.setNotOKShootDriver();
    m_shooter.shootCargosRunning = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_transport.cargosShot >= 5 || inAuto && m_transport.cargosShot >= 3;
  }

  public void releaseOneCargo() {
    if (!cargoReleased) {

      cargoReleased = true;
      cargoReleasedStartTime = Timer.getFPGATimestamp();
    }

    if (cargoReleased && Timer.getFPGATimestamp() > cargoReleasedStartTime + m_transport.cargoPassTime) {

      cargoReleasedStartTime = 0;

      m_transport.cargoAvailable = true;

      cargoReleased = false;
    }

  }
}