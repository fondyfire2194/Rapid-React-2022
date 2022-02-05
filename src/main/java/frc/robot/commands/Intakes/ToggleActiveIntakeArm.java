// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakesSubsystem;

public class ToggleActiveIntakeArm extends CommandBase {

  private IntakesSubsystem m_intake;
  private boolean m_useFrontIntake;

  public ToggleActiveIntakeArm(IntakesSubsystem intake, boolean useFrontIntake) {
    m_intake = intake;
    m_useFrontIntake = useFrontIntake;
  }

  public void initialize() {

  }

  @Override

  public void execute() {

    if (m_useFrontIntake)

      if (m_intake.getFrontArmRaised()) {
        m_intake.lowerFrontArm();
      } else {
        m_intake.raiseFrontArm();
      }

    else {

      if (m_intake.getRearArmRaised()) {
        m_intake.lowerRearArm();
      } else {
        m_intake.raiseRearArm();
      }
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
