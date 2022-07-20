// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class IntakeToShootPosition extends CommandBase {

  private IntakesSubsystem m_intake;
  private CargoTransportSubsystem m_transport;
  /** Creates a new IntakeToShootPosition. */
  public IntakeToShootPosition(IntakesSubsystem intake, CargoTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport=transport;
    m_intake=intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.lowerActiveArm();
    m_intake.runActiveIntakeMotor();
    m_transport.intakeCargo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRearIntakeMotor();
    m_intake.stopFrontIntakeMotor();
    m_intake.raiseRearArm();
    m_intake.raiseFrontArm();
    m_transport.stopLowerRoller();
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transport.getCargoAtShoot();
  }
}
