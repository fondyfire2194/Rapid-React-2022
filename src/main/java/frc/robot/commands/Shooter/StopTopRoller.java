// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevShooterSubsystem;

public class StopTopRoller extends CommandBase {
  /** Creates a new RunRollers. */
  private final RevShooterSubsystem m_shooter;

  public StopTopRoller(RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_shooter.haltTopRoller=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.stopTopRoller();
    m_shooter.haltTopRollerMotor = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.haltTopRollerMotor = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getTopRPM()) < 10;
  }
}
