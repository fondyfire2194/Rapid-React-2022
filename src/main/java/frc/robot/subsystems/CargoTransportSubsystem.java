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

  private AnalogInput leftChannelBallDetect;

  private AnalogInput ballAtShootDetect;// next to be released for shooting

  // private AnalogInput ballBlockLeft;// wait to go into shoot position

  private double leftHoldPosition;
  private double leftReleasePosition;

  public boolean startRollers;
  public double cargoPassTime = .25;
  public boolean rollersAtSpeed;
  public double rollerSpeed;
  public boolean haltRollers;
  public boolean haltBelts;
  public int cargosShot;
  public boolean leftArmDown;

  public boolean noBallatLeftForOneSecond;

  public boolean noBallatShooterForOneSecond;

  private double noBallatShooterTime;

  private double noBallatLeftTime;

  private double ballDetectedVolts = 2.75;

  public boolean cargoAvailable = true;
  private double ballTravelTime = 1;
  public int cargosToBeShot = 3;

  public CargoTransportSubsystem() {

    m_frontRollerMotor = new WPI_TalonSRX(CANConstants.FRONT_ROLLER);
    m_rearRollerMotor = new WPI_TalonSRX(CANConstants.REAR_ROLLER);


    leftChannelBallDetect = new AnalogInput(1);

    // ballBlockLeft = new AnalogInput(0);

    ballAtShootDetect = new AnalogInput(2);

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

    if (getBallAtShoot()) {
      noBallatShooterForOneSecond = false;
      noBallatShooterTime = 0;
    }

    if (!getBallAtShoot() && noBallatShooterTime == 0) {
      noBallatShooterTime = Timer.getFPGATimestamp();
    }

    if (!getBallAtShoot() && noBallatShooterTime != 0
        && Timer.getFPGATimestamp() > noBallatShooterTime + ballTravelTime) {
      noBallatShooterForOneSecond = true;
    }
    // left cargo detection
    if (getBallAtLeft()) {
      noBallatLeftForOneSecond = false;
      noBallatLeftTime = 0;
    }

    if (!getBallAtShoot() && noBallatLeftTime == 0) {
      noBallatLeftTime = Timer.getFPGATimestamp();
    }

    if (!getBallAtShoot() && noBallatLeftTime != 0 && Timer.getFPGATimestamp() > noBallatLeftTime + ballTravelTime) {
      noBallatLeftForOneSecond = true;
    }

  }

  public boolean checkCAN() {

    frontRollerMotorConnected = m_frontRollerMotor.getFirmwareVersion() != -1;
    rearRollerMotorConnected = m_rearRollerMotor.getFirmwareVersion() != -1;
    allConnected = leftBeltMotorConnected && rightBeltMotorConnected && frontRollerMotorConnected
        && rearRollerMotorConnected;

    return leftBeltMotorConnected && rightBeltMotorConnected && frontRollerMotorConnected && rearRollerMotorConnected;

  }

  
  public boolean getBallAtLeft() {
    return leftChannelBallDetect.getVoltage() > ballDetectedVolts;

  }

  public boolean getBallAtShoot() {
    return ballAtShootDetect.getVoltage() > ballDetectedVolts;
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
