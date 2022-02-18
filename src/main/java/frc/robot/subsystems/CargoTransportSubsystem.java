/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class CargoTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CargoTransport.
   */

  private final WPI_TalonSRX m_frontRollerMotor;
  private final WPI_TalonSRX m_rearRollerMotor;

  public List<BaseTalon> transportTalons;

  public boolean leftBeltMotorConnected;
  public boolean rightBeltMotorConnected;
  public boolean frontRollerMotorConnected;
  public boolean rearRollerMotorConnected;
  public boolean allConnected;

  private AnalogInput leftChannelCargoDetect;

  private AnalogInput cargoAtShootDetect;// next to be released for shooting

  // private AnalogInput CargoBlockLeft;// wait to go into shoot position


  public boolean startRollers;
  public double cargoPassTime = .25;
  public boolean rollersAtSpeed;
  public double rollerSpeed;
  public boolean haltRollers;
  public boolean haltBelts;
  public int cargosShot;
  public boolean leftArmDown;

  public boolean noCargoatLeftForOneSecond;

  public boolean noCargoatShooterForOneSecond;

  private double noCargoatShooterTime;

  private double noCargoatLeftTime;

  private double CargoDetectedVolts = 2.75;

  public boolean cargoAvailable = true;
  private double CargoTravelTime = 1;
  public int cargosToBeShot = 3;

  public CargoTransportSubsystem() {

    m_frontRollerMotor = new WPI_TalonSRX(CANConstants.FRONT_ROLLER);
    m_rearRollerMotor = new WPI_TalonSRX(CANConstants.REAR_ROLLER);


    leftChannelCargoDetect = new AnalogInput(1);

    // CargoBlockLeft = new AnalogInput(0);

    cargoAtShootDetect = new AnalogInput(2);

    m_frontRollerMotor.configFactoryDefault();
    m_rearRollerMotor.configFactoryDefault();

    m_frontRollerMotor.setNeutralMode(NeutralMode.Brake);
    m_rearRollerMotor.setNeutralMode(NeutralMode.Brake);

    setFrontRollerBrakeOn(true);
    setRearRollerBrakeOn(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getCargoAtShoot()) {
      noCargoatShooterForOneSecond = false;
      noCargoatShooterTime = 0;
    }

    if (!getCargoAtShoot() && noCargoatShooterTime == 0) {
      noCargoatShooterTime = Timer.getFPGATimestamp();
    }

    if (!getCargoAtShoot() && noCargoatShooterTime != 0
        && Timer.getFPGATimestamp() > noCargoatShooterTime + CargoTravelTime) {
      noCargoatShooterForOneSecond = true;
    }
    // left cargo detection
    if (getCargoAtLeft()) {
      noCargoatLeftForOneSecond = false;
      noCargoatLeftTime = 0;
    }

    if (!getCargoAtShoot() && noCargoatLeftTime == 0) {
      noCargoatLeftTime = Timer.getFPGATimestamp();
    }

    if (!getCargoAtShoot() && noCargoatLeftTime != 0 && Timer.getFPGATimestamp() > noCargoatLeftTime + CargoTravelTime) {
      noCargoatLeftForOneSecond = true;
    }

  }

  public boolean checkCAN() {

    frontRollerMotorConnected = m_frontRollerMotor.getFirmwareVersion() != -1;
    rearRollerMotorConnected = m_rearRollerMotor.getFirmwareVersion() != -1;
    allConnected = leftBeltMotorConnected && rightBeltMotorConnected && frontRollerMotorConnected
        && rearRollerMotorConnected;

    return leftBeltMotorConnected && rightBeltMotorConnected && frontRollerMotorConnected && rearRollerMotorConnected;

  }

  
  public boolean getCargoAtLeft() {
    return leftChannelCargoDetect.getVoltage() > CargoDetectedVolts;

  }

  public boolean getCargoAtShoot() {
    return cargoAtShootDetect.getVoltage() > CargoDetectedVolts;
  }

  public void runFrontRollerMotor(double speed) {
    m_frontRollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getFrontRoller() {
    return m_frontRollerMotor.getMotorOutputPercent();
  }

  public void stopFrontRollerMotor() {
    m_frontRollerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setFrontRollerBrakeOn(boolean on) {
    if (on) {
      m_frontRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_frontRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void runRearRollerMotor(double speed) {
    m_rearRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public double getRearRoller() {
    return m_rearRollerMotor.getMotorOutputPercent();
  }

  public void stopRearRollerMotor() {
    m_rearRollerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopRollers() {
    m_frontRollerMotor.stopMotor();
    m_rearRollerMotor.stopMotor();
  }

  public void setRearRollerBrakeOn(boolean on) {
    if (on) {
      m_rearRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_rearRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getFrontRollerMotorAmps() {
    return m_rearRollerMotor.getStatorCurrent();
  }

  public double getRearRollerMotorAmps() {
    return m_frontRollerMotor.getStatorCurrent();
  }

  public void simulationInit() {

  }

}
