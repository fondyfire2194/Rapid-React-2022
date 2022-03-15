// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakesSubsystem;

public class StopActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private int loopctr;

  public StopActiveIntake(IntakesSubsystem intake) {
    m_intake = intake;

    addRequirements(m_intake);
  }

  public void initialize() {
    loopctr = 0;
  }

  @Override

  public void execute() {
    loopctr++;
    m_intake.stopFrontIntakeMotor();
    m_intake.raiseFrontArm();

    m_intake.stopRearIntakeMotor();
    m_intake.raiseRearArm();

  }

  @Override
  public void end(boolean interrupted) {

    m_intake.stopFrontIntakeMotor();
    m_intake.raiseFrontArm();

    m_intake.stopRearIntakeMotor();
    m_intake.raiseRearArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
