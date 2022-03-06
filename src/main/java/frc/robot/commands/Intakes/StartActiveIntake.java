// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;

public class StartActiveIntake extends CommandBase {

  private IntakesSubsystem m_intake;

  private CargoTransportSubsystem m_transport;

  private double m_speed;

  private double m_rollerSpeed;

  private boolean twoCargoOnBoard;

  private boolean cargoAtLowerRoller;

  public StartActiveIntake(IntakesSubsystem intake, double iSpeed, CargoTransportSubsystem transport, double rSpeed) {
    m_intake = intake;
    m_transport = transport;
    m_speed = iSpeed;
    m_rollerSpeed = rSpeed;
    addRequirements(m_intake);
  }

  public void initialize() {

    // m_transport.configLowerCurrentLimit(5, 100,true);

  }

  @Override

  public void execute() {

    if (!cargoAtLowerRoller) {

      int speed = 200;

      m_transport.runLowerAtVelocity(speed);

    } else {


      m_transport.stopLowerRoller();
    }

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

    m_transport.stopLowerRoller();

    m_transport.haltLowerRollerMotor=true;

    // m_transport.configLowerCurrentLimit(50, 1000, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
