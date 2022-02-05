// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntakeMotor extends CommandBase {

  private IntakesSubsystem m_intake;
  private double m_speed;

  public RunActiveIntakeMotor(IntakesSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(m_intake);
  }

  public void initialize() {

  }

  @Override

  public void execute() {

    if (m_intake.useFrontIntake)

      m_intake.runFrontIntakeMotor(m_speed);

    else {

      m_intake.runRearIntakeMotor(m_speed);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
