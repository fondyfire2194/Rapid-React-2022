// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakesSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFrontIntakeActive extends InstantCommand {

  private IntakesSubsystem m_intake;
  private boolean m_on;

  public SetFrontIntakeActive(IntakesSubsystem intake, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_on = on;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_on)
      m_intake.setFrontActive();
    else
      m_intake.setRearActive();
  }
}
