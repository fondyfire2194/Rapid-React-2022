/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevShooterSubsystem;

public class JogShooter extends CommandBase {
  /**
   * Creates a new StartShooter.
   */
  private RevShooterSubsystem m_shooter;
  private final Supplier<Double> m_xaxisSpeedSupplier;

  public JogShooter(RevShooterSubsystem shooter, Supplier<Double> xaxisSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_xaxisSpeedSupplier.get()) < .1)
      m_shooter.moveManually(0);
    else
      m_shooter.moveManually(m_xaxisSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
