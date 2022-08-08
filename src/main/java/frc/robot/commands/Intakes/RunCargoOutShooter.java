// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class RunCargoOutShooter extends CommandBase {
  /** Creates a new RunCargoOutShooter. */
  private RevShooterSubsystem m_shooter;
  private IntakesSubsystem m_intake;
  private CargoTransportSubsystem m_transport;
  private double m_rpm;

  public RunCargoOutShooter(RevShooterSubsystem shooter, IntakesSubsystem intake, CargoTransportSubsystem transport,double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_intake = intake;
    m_transport = transport;
    m_rpm=rpm;
    addRequirements(m_intake, m_transport, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  
    m_transport.wrongCargoColor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.spinAtRPM(m_rpm);

    m_shooter.runTopAtVelocity(800);

    m_transport.runLowerAtVelocity(800);

    // m_intake.runRearIntakeMotor();
    // m_intake.runFrontIntakeMotor();

    m_intake.simCargoAtFrontIntake = false;
    m_intake.simCargoAtRearIntake = false;   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.stopTopRoller();
    m_intake.stopRearIntakeMotor();
    m_intake.stopFrontIntakeMotor();
    m_transport.stopLowerRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
