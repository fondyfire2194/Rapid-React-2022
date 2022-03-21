// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakesSubsystem;

public class DelayOffIntake extends CommandBase {
  /** Creates a new DelayOffIntake. */
  private IntakesSubsystem m_intake;
  private double m_startTime;

  public DelayOffIntake(IntakesSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_intake.endIntakeCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.endIntakeCommand = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_startTime != 0 && Timer.getFPGATimestamp() > m_startTime + 1;
  }
}
