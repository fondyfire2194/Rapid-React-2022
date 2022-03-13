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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public ColorSensorV3 cargoAtShootsensor = new ColorSensorV3(i2cPort);

  private AnalogInput atShootCargoDetect;

  private double cargoAtShootVolts = 2.2;

  private double cargoHasLeftShootVolts = 1.2;

  private double cargoAtShootActiveVolts = cargoAtShootVolts;

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

    m_lowerRollerMotor.setIdleMode(IdleMode.kBrake);

    setLowerRollerBrakeOn(true);

    calibrateLowerPID();

    atShootCargoDetect = new AnalogInput(1);

    atShootCargoDetect.setAverageBits(12);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("CARSGVOL", atShootCargoDetect.getAverageVoltage());

    if (getCargoAtShoot()) {

      cargoAtShootActiveVolts = cargoHasLeftShootVolts;
    }
    if (!getCargoAtShoot()) {

      cargoAtShootActiveVolts = cargoAtShootVolts;
    }
  }

  public boolean checkCAN() {

    lowerRollerMotorConnected = m_lowerRollerMotor.getFirmwareVersion() != -1;

    return lowerRollerMotorConnected;

  }

  public void reverseLowRoller() {
    
    m_lowerRollerMotor.set(-.2);

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

    return getCargoAtShoot() && ((getAllianceBlue() && getCargoIsBlue())

        || (!getAllianceBlue() && getCargoIsRed()));
  }

  public boolean getCargoAtShoot() {

    return atShootCargoDetect.getAverageVoltage() > cargoAtShootActiveVolts;

  }

  public void intakeLowerRollerMotor() {

    double rpm = Pref.getPref("LowRollIntakeRPM");

    if (!getCargoAtShoot()) {

      runLowerAtVelocity(rpm);

    }

    else {

      runLowerAtVelocity(0);

    }

  }

  public void releaseCargo() {

    double rpm = Pref.getPref("LowRollReleaseRPM");

    if (getCargoAtShoot()) {

      runLowerAtVelocity(rpm);
    }
  }

  public void runLowerAtVelocity(double rpm) {

    lowerRequiredRPM = rpm;

    m_lowerPID.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
  }

  public void reverseLowerRoller() {

    m_lowerPID.setReference(-200, ControlType.kVelocity, VELOCITY_SLOT);
  }

  public boolean getLowerRollerAtSpeed() {

    return Math.abs(lowerRequiredRPM - getLowerRPM()) < (lowerRequiredRPM * .05);// getmps is -
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
    lowerRequiredRPM = 0;

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
    double p = Pref.getPref("Rollers_kP");
    double i = 0;
    double d = Pref.getPref("Rollers_kD");
    double f = .0009;// 1/1000 (10,000 rpm through 10:1 gearing)
    double kIz = 0;
    double acc = .5;

    m_lowerPID.setP(p, VELOCITY_SLOT);

    m_lowerPID.setI(i, VELOCITY_SLOT);

    m_lowerPID.setD(d, VELOCITY_SLOT);

    m_lowerPID.setFF(f, VELOCITY_SLOT);

    m_lowerPID.setIZone(kIz, VELOCITY_SLOT);

    m_lowerPID.setOutputRange(-0.5, 0.5, VELOCITY_SLOT);

    m_lowerRollerMotor.setClosedLoopRampRate(acc);

  }

}
