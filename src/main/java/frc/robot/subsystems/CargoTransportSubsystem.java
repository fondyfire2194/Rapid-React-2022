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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RevColorSensor;
import frc.robot.Robot;
import frc.robot.Constants.CANConstants;

public class CargoTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CargoTransport.
   */

  private final WPI_TalonSRX m_topRollerMotor;
  private final WPI_TalonSRX m_lowerRollerMotor;

  public List<BaseTalon> transportTalons;

  public boolean leftBeltMotorConnected;
  public boolean rightBeltMotorConnected;
  public boolean topRollerMotorConnected;
  public boolean lowerRollerMotorConnected;
  public boolean allConnected;

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

  public boolean cargoAvailable = true;

  private double CargoTravelTime = 1;

  private boolean cargoAtShoot;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public ColorSensorV3 cargoAtShootsensor = new ColorSensorV3(i2cPort);
  private int cargoSensedLevel = 180;
  private RevColorSensor rcs = new RevColorSensor(cargoAtShootsensor);

  public CargoTransportSubsystem() {

    m_topRollerMotor = new WPI_TalonSRX(CANConstants.TOP_ROLLER);
    m_lowerRollerMotor = new WPI_TalonSRX(CANConstants.LOWER_ROLLER);

    m_topRollerMotor.configFactoryDefault();
    m_lowerRollerMotor.configFactoryDefault();

    m_topRollerMotor.setNeutralMode(NeutralMode.Brake);
    m_lowerRollerMotor.setNeutralMode(NeutralMode.Brake);

    setTopRollerBrakeOn(true);
    setLowerRollerBrakeOn(true);
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

    noCargoatLeftTime = Timer.getFPGATimestamp();

    if (getCargoAtShoot() == false && noCargoatShooterForOneSecond) {
      noCargoatLeftForOneSecond = true;
    }
  }

  public boolean checkCAN() {

    topRollerMotorConnected = m_topRollerMotor.getFirmwareVersion() != -1;
    lowerRollerMotorConnected = m_lowerRollerMotor.getFirmwareVersion() != -1;
    allConnected = topRollerMotorConnected
        && lowerRollerMotorConnected;

    return topRollerMotorConnected && lowerRollerMotorConnected;

  }

  public boolean getCargoIsBlue() {
    return rcs.getIsBlue();
  }

  public boolean getCargoIsRed() {
    return rcs.getIsRed();
  }

  public boolean getAllianceBlue() {
    return Robot.getAllianceColor();
  }

  public boolean getCargoAllianceMatch() {
    return getAllianceBlue() && getCargoAtShoot() && getCargoIsBlue()
        || !getAllianceBlue() && getCargoAtShoot() && getCargoIsRed();
  }

  public boolean getCargoAtShoot() {
    return cargoAtShootsensor.getProximity() > cargoSensedLevel;
  }

  public void runTopRollerMotor(double speed) {
    m_topRollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getTopRoller() {
    return m_topRollerMotor.getMotorOutputPercent();
  }

  public void stopTopRollerMotor() {
    m_topRollerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setTopRollerBrakeOn(boolean on) {
    if (on) {
      m_topRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_topRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void runLowerRollerMotor(double speed) {
    m_lowerRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public double getLowerRoller() {
    return m_lowerRollerMotor.getMotorOutputPercent();
  }

  public void stopLowerRollerMotor() {
    m_lowerRollerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopRollers() {
    m_topRollerMotor.stopMotor();
    m_lowerRollerMotor.stopMotor();
  }

  public void setLowerRollerBrakeOn(boolean on) {
    if (on) {
      m_lowerRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_lowerRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getTopRollerMotorAmps() {
    return m_topRollerMotor.getStatorCurrent();
  }

  public double getLowerRollerMotorAmps() {
    return m_lowerRollerMotor.getStatorCurrent();
  }

  public void simulationInit() {

  }

}
