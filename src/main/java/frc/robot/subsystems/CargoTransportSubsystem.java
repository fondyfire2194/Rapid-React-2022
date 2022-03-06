/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.RevColorSensor;
import frc.robot.Robot;

public class CargoTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CargoTransport.
   */

  private final CANSparkMax m_topRollerMotor;
  private final CANSparkMax m_lowerRollerMotor;

  public boolean topRollerMotorConnected;
  public boolean lowerRollerMotorConnected;

  public boolean haltTopRollerMotor;
  public boolean haltLowerRollerMotor;

  public boolean allConnected;

  // private AnalogInput CargoBlockLeft;// wait to go into shoot position

  
  public double cargoPassTime = .25;
 
  
  public int cargosShot;

  public boolean leftArmDown;

  public boolean noCargoAtShooterForOneSecond;

  private double noCargoAtShooterTime;

  public boolean cargoAvailable = true;

  private double CargoTravelTime = 1;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public ColorSensorV3 cargoAtShootsensor = new ColorSensorV3(i2cPort);

  private int cargoSensedLevel = 180;

  private RevColorSensor rcs = new RevColorSensor(cargoAtShootsensor);

  private SparkMaxPIDController m_lowerPID;
  private SparkMaxPIDController m_topPID;

  private RelativeEncoder m_lowerEncoder;
  private RelativeEncoder m_topEncoder;

  public double topRequiredRPM;
  public double lowerRequiredRPM;

  public CargoTransportSubsystem() {

    m_topRollerMotor = new CANSparkMax(CANConstants.TOP_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_lowerRollerMotor = new CANSparkMax(CANConstants.LOWER_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_topPID = m_topRollerMotor.getPIDController();
    m_lowerPID = m_lowerRollerMotor.getPIDController();

    m_topEncoder = m_topRollerMotor.getEncoder();
    m_lowerEncoder = m_lowerRollerMotor.getEncoder();

    m_topEncoder.setPositionConversionFactor(36);
    m_lowerEncoder.setPositionConversionFactor(36);

    m_topEncoder.setVelocityConversionFactor(36);
    m_lowerEncoder.setVelocityConversionFactor(36);

    m_topRollerMotor.restoreFactoryDefaults();
    m_lowerRollerMotor.restoreFactoryDefaults();

    m_topRollerMotor.setInverted(true);
    m_lowerRollerMotor.setInverted(true);

    m_lowerRollerMotor.setSmartCurrentLimit(10, 20);

    m_topRollerMotor.setIdleMode(IdleMode.kBrake);

    m_lowerRollerMotor.setIdleMode(IdleMode.kBrake);

    setTopRollerBrakeOn(true);
    setLowerRollerBrakeOn(true);

    calibrateLowerPID();
    calibrateTopPID();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getCargoAtShoot()) {

      noCargoAtShooterForOneSecond = false;

      noCargoAtShooterTime = 0;
    }

    if (!getCargoAtShoot() && noCargoAtShooterTime == 0) {
      noCargoAtShooterTime = Timer.getFPGATimestamp();
    }

    if (!getCargoAtShoot() && noCargoAtShooterTime != 0

        && Timer.getFPGATimestamp() > noCargoAtShooterTime + CargoTravelTime) {

      noCargoAtShooterForOneSecond = true;
    }
  }

  public boolean checkCAN() {

    topRollerMotorConnected = m_topRollerMotor.getFirmwareVersion() != -1;
    lowerRollerMotorConnected = m_lowerRollerMotor.getFirmwareVersion() != -1;
    allConnected = topRollerMotorConnected
        && lowerRollerMotorConnected;

    return topRollerMotorConnected && lowerRollerMotorConnected;

  }

  public void setTopRollerBrakeOn(boolean on) {
    if (on) {
      m_topRollerMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_topRollerMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setLowerRollerBrakeOn(boolean on) {
    if (on) {
      m_lowerRollerMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_lowerRollerMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public boolean getCargoIsBlue() {
    return rcs.getIsBlue();
  }

  public boolean getCargoIsRed() {
    return rcs.getIsRed();
  }

  public boolean getAllianceBlue() {
    return Robot.getAllianceColorBlue();
  }

  public boolean getCargoAllianceMatch() {
    return getAllianceBlue() && getCargoAtShoot() && getCargoIsBlue()
        || !getAllianceBlue() && getCargoAtShoot() && getCargoIsRed();
  }

  public boolean getCargoAtShoot() {

    return cargoAtShootsensor.getProximity() > cargoSensedLevel;

  }

  public void intakeLowerRollerMotor(double speed) {

    if (!getCargoAtShoot()) {

      runLowerRollerMotor(speed);

    }

  }

  public void runTopAtVelocity(double speed) {

    m_topPID.setReference(speed, ControlType.kVelocity, 0);
  }

  public void moveManually(double speed) {

  }

  public void runLowerAtVelocity(double speed) {

    m_topPID.setReference(speed, ControlType.kVelocity, 0);
  }

  public boolean getTopRollerAtSpeed() {
    return Math.abs(topRequiredRPM + getTopRPM()) < (topRequiredRPM * .05);// getmps is -
  }

  public boolean getLowerRollerAtSpeed() {
    return Math.abs(lowerRequiredRPM + getLowerRPM()) < (lowerRequiredRPM * .05);// getmps is -
  }

  public double getTopRPM() {
    return m_topEncoder.getVelocity();
  }

  public double getLowerRPM() {
    return m_lowerEncoder.getVelocity();
  }

  public double getTopRoller() {
    return m_topRollerMotor.get();
  }

  public void stopTopRollerMotor() {
    m_topRollerMotor.set(0);
  }

  public void runLowerRollerMotor(double speed) {
    m_lowerRollerMotor.set(-speed);
  }

  public double getLowerRoller() {
    return m_lowerRollerMotor.get();
  }

  public void stopLowerRoller() {
    m_lowerRollerMotor.stopMotor();
  }

  public void stopTopRoller() {
    m_topRollerMotor.stopMotor();
  }

  public double getTopRollerMotorAmps() {
    return m_topRollerMotor.getOutputCurrent();
  }

  public double getLowerRollerMotorAmps() {
    return m_lowerRollerMotor.getOutputCurrent();
  }

  public void configLowerCurrentLimit(int amps, int time_ms, boolean enable) {
    int stallLimit = 25;
    int freeLimit = 30;
    m_lowerRollerMotor.setSmartCurrentLimit(stallLimit, freeLimit);

  }

  public void simulationInit() {

  }

  public void calibrateTopPID() {
    double p = 0.001;
    double i = 0;
    double d = 0;
    double f = 1 / 760;
    double kIz = 0;
    double acc = 1;

    m_topPID.setP(p, 0);

    m_topPID.setI(i, 0);

    m_topPID.setD(d, 0);

    m_topPID.setFF(f, 0);

    m_topPID.setIZone(kIz, 0);

    m_topPID.setOutputRange(-0.5, 0.5, 0);

    m_topRollerMotor.setClosedLoopRampRate(acc);

  }

  public void calibrateLowerPID() {
    double p = 0.001;
    double i = 0;
    double d = 0;
    double f = 1 / 760;
    double kIz = 0;
    double acc = 1;

    m_lowerPID.setP(p, 0);

    m_lowerPID.setI(i, 0);

    m_lowerPID.setD(d, 0);

    m_lowerPID.setFF(f, 0);

    m_lowerPID.setIZone(kIz, 0);

    m_lowerPID.setOutputRange(-0.5, 0.5, 0);

    m_lowerRollerMotor.setClosedLoopRampRate(acc);

  }

}
