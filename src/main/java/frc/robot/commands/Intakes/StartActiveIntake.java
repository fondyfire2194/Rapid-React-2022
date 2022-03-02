// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakesSubsystem;

public class StartActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private double m_speed;

  public StartActiveIntake(IntakesSubsystem intake, double speed) {
    m_intake = intake;

    m_speed = speed;
    addRequirements(m_intake);
  }

  public void initialize() {

  }

  @Override

  public void execute() {

    if (m_intake.useFrontIntake) {

      m_intake.runFrontIntakeMotor(m_speed);

      m_intake.lowerFrontArm();

    } else {

      m_intake.runRearIntakeMotor(m_speed);
      
      m_intake.lowerRearArm();
    }

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
    return true;

  }
}
