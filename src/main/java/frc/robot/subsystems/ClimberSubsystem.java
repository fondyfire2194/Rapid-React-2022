// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;


public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  final WPI_TalonSRX m_climberMotor = new WPI_TalonSRX(CANConstants.CLIMB_MOTOR);
  final DoubleSolenoid m_climberArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  final DoubleSolenoid m_climberRatchet = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,6, 7);

  public boolean climberMotorConnected;

  public ClimberSubsystem() {

    m_climberMotor.configFactoryDefault();

    m_climberMotor.setNeutralMode(NeutralMode.Brake);
    
    
    lockRatchet();

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

  public void raiseArm() {
    m_climberArm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lowerArm() {
    m_climberArm.set(DoubleSolenoid.Value.kForward);
  }

  public boolean getArmRaised() {
    return m_climberArm.get() == DoubleSolenoid.Value.kReverse;
  }

  public boolean getArmLowered() {
    return m_climberArm.get() == DoubleSolenoid.Value.kForward;
  }

  public void lockRatchet() {
    m_climberRatchet.set(DoubleSolenoid.Value.kForward);
  }

  public void unlockRatchet() {
    m_climberRatchet.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean getRatchetLocked() {
    return m_climberRatchet.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getRatchetUnlocked() {
    return m_climberRatchet.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public boolean checkCAN() {
    return climberMotorConnected = m_climberMotor.getFirmwareVersion() != -1;

  }

  public void simulationInit() {
  
  }

  public void simulationPeriodic() {
   
  }
}
