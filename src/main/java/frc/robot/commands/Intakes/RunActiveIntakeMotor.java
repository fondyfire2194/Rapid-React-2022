// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class RunActiveIntakeMotor extends CommandBase {

  private IntakesSubsystem m_intake;
  private CargoTransportSubsystem m_transport;

  private double lr_rpm;

  public RunActiveIntakeMotor(IntakesSubsystem intake, CargoTransportSubsystem transport) {
    m_intake = intake;
    m_transport = transport;

    addRequirements(m_intake);
  }

  public void initialize() {
    
    lr_rpm = Pref.getPref("LowRollIntakeRPM");
  }

  @Override

  public void execute() {

    m_intake.runActiveIntakeMotor();

    m_transport.intakeLowerRollerMotor(lr_rpm);

  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopFrontIntakeMotor();
    m_intake.stopRearIntakeMotor();
    m_transport.stopLowerRoller();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
