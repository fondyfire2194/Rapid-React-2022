// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class StartActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private CargoTransportSubsystem m_transport;
  private double lr_rpm;

  public StartActiveIntake(IntakesSubsystem intake, CargoTransportSubsystem transport) {
    m_intake = intake;
    m_transport = transport;

    addRequirements(m_intake);

    lr_rpm = Pref.getPref("LowRollIntakeRPM");
  }

  public void initialize() {

  }

  @Override

  public void execute() {

    m_intake.lowerActiveArm();

    m_intake.runActiveIntakeMotor();

  
  }

  @Override
  public void end(boolean interrupted) {

    m_intake.stopFrontIntakeMotor();

    m_intake.raiseFrontArm();

    m_intake.stopRearIntakeMotor();

    m_intake.raiseRearArm();

    m_transport.stopLowerRoller();

    Shuffleboard.selectTab("Intake");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
