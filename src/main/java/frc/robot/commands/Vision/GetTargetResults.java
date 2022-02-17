// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.GetTarget;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class GetTargetResults extends CommandBase {
  private GetTarget m_target;
  private RevTurretSubsystem m_turret;
  private double angleChange;

  /** Creates a new GetTargetResults. */
  public GetTargetResults(GetTarget target, RevTurretSubsystem turret) {
    m_target = target;
    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int targetChange = m_target.calculateTargetX(); // returns x to be used later
    angleChange = m_target.getTargetAngle(targetChange);
    m_turret.targetHorizontalOffset = angleChange;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
