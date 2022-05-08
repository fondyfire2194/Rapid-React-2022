// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  final WPI_TalonSRX m_climberMotor = new WPI_TalonSRX(CANConstants.CLIMB_MOTOR);

  public boolean climberMotorConnected;

  public ClimberSubsystem() {

    m_climberMotor.configFactoryDefault();

    m_climberMotor.setNeutralMode(NeutralMode.Brake);

    setStatusFramePeriods();
  }

  public void runMotor(double speed) {

    m_climberMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotor() {

    m_climberMotor.stopMotor();
  }

  public double getMotorAmps() {
    return m_climberMotor.getStatorCurrent();
  }

  public double getMotorOut() {
    return m_climberMotor.getMotorOutputPercent();
  }

  public void setBrakeOn(boolean on) {
    if (on) {
      m_climberMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_climberMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber", m_climberMotor.getStatorCurrent());
  }

  public boolean checkCAN() {
    return climberMotorConnected = m_climberMotor.getFirmwareVersion() != -1;

  }

  public void setStatusFramePeriods() {

    m_climberMotor.setStatusFramePeriod(2, 200);

    m_climberMotor.setStatusFramePeriod(3, 200);
  }

}
