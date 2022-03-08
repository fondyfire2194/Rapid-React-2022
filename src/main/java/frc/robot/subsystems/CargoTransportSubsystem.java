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
import frc.robot.Pref;
import frc.robot.RevColorSensor;
import frc.robot.Robot;

public class CargoTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CargoTransport.
   */

  private final CANSparkMax m_lowerRollerMotor;

  public boolean lowerRollerMotorConnected;

  public boolean haltLowerRollerMotor;

  public boolean allConnected;

  public boolean noCargoAtShooterForOneSecond;

  private double noCargoAtShooterTime;

  private double CargoTravelTime = 1;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public ColorSensorV3 cargoAtShootsensor = new ColorSensorV3(i2cPort);

  private int cargoSensedLevel = 800;

  public final int VELOCITY_SLOT = 0;
  private static final int SMART_MOTION_SLOT = 1;
  public final int POSITION_SLOT = 2;

  private RevColorSensor rcs = new RevColorSensor(cargoAtShootsensor);

  private SparkMaxPIDController m_lowerPID;

  private RelativeEncoder m_lowerEncoder;

  public double lowerRequiredRPM;

  public boolean lowerRollerPositioning;

  private double m_startTime;

  public CargoTransportSubsystem() {

    m_lowerRollerMotor = new CANSparkMax(CANConstants.LOWER_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_lowerPID = m_lowerRollerMotor.getPIDController();

    m_lowerEncoder = m_lowerRollerMotor.getEncoder();

    m_lowerRollerMotor.restoreFactoryDefaults();

    m_lowerEncoder.setPositionConversionFactor(.1);

    m_lowerEncoder.setVelocityConversionFactor(.1);

    m_lowerRollerMotor.setInverted(false);

    // m_lowerRollerMotor.setSmartCurrentLimit(50, 50);

    m_lowerRollerMotor.setIdleMode(IdleMode.kBrake);

    setLowerRollerBrakeOn(true);

    calibrateLowerPID();

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

    lowerRollerMotorConnected = m_lowerRollerMotor.getFirmwareVersion() != -1;

    return lowerRollerMotorConnected;

  }

  public void positionLowerRoller(double distance) {
    m_lowerPID.setReference(distance, ControlType.kPosition, VELOCITY_SLOT);
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

  public void intakeLowerRollerMotor() {

    if (!getCargoAtShoot()) {

      runLowerAtVelocity();

    }

    else {

      stopLowerRoller();

    }

  }

  public void runLowerAtVelocity() {

    double speed = Pref.getPref("LowerRollerSpeed");

    m_lowerPID.setReference(speed, ControlType.kVelocity, VELOCITY_SLOT);
  }

  public boolean getLowerRollerAtSpeed() {

    return Math.abs(lowerRequiredRPM + getLowerRPM()) < (lowerRequiredRPM * .05);// getmps is -
  }

  public double getLowerRPM() {

    return m_lowerEncoder.getVelocity();
  }

  public void runLowerRollerMotor(double speed) {
    m_lowerRollerMotor.set(speed);
  }

  public double getLowerRoller() {
    return m_lowerRollerMotor.get();
  }

  public void stopLowerRoller() {
    m_lowerRollerMotor.stopMotor();
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

  public void calibrateLowerPID() {
    double p = 0.001;
    double i = 0;
    double d = 0;
    double f = .001;// 1/1000 (10,000 rpm through 10:1 gearig)
    double kIz = 0;
    double acc = 1000;

    m_lowerPID.setP(p, VELOCITY_SLOT);

    m_lowerPID.setI(i, VELOCITY_SLOT);

    m_lowerPID.setD(d, VELOCITY_SLOT);

    m_lowerPID.setFF(f, VELOCITY_SLOT);

    m_lowerPID.setIZone(kIz, VELOCITY_SLOT);

    m_lowerPID.setOutputRange(-0.5, 0.5, VELOCITY_SLOT);

    m_lowerRollerMotor.setClosedLoopRampRate(acc);

  }

  public void releaseCargo() {

    if (getCargoAtShoot())
    
      runLowerAtVelocity();
  }

}
