/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Sim.CANEncoderSim;
import frc.robot.Sim.CANSparkMaxWithSim;
import frc.robot.Pref;
import frc.robot.RevColorSensor;
import frc.robot.Robot;

public class CargoTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CargoTransport.
   */

  private final CANSparkMaxWithSim m_lowerRollerMotor;

  private CANEncoderSim lowRollEncoderSim;

  private FlywheelSim lowRollerSim;

  public boolean lowerRollerMotorConnected;

  public boolean haltLowerRollerMotor;

  public boolean allConnected;

  public boolean noCargoAtShooterForOneSecond;

  public AnalogInput cargoAboveLowRoll = new AnalogInput(1);

  public DigitalInput cargoSensor = new DigitalInput(0);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public ColorSensorV3 cargoAtShootsensor = new ColorSensorV3(i2cPort);

  public final int VELOCITY_SLOT = 0;
  private static final int SMART_MOTION_SLOT = 1;
  public final int POSITION_SLOT = 2;

  private RevColorSensor rcs = new RevColorSensor(cargoAtShootsensor);

  private SparkMaxPIDController m_lowerPID;

  private RelativeEncoder m_lowerEncoder;

  public double lowerRequiredRPM;

  public boolean lowerRollerPositioning;

  public boolean wrongCargoColor;

  public boolean latchCargoAtShoot;

  public boolean cargoIsBlue;

  public boolean cargoIsRed;

  public CargoTransportSubsystem() {

    m_lowerRollerMotor = new CANSparkMaxWithSim(CANConstants.LOWER_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_lowerPID = m_lowerRollerMotor.getPIDController();

    m_lowerEncoder = m_lowerRollerMotor.getEncoder();

    m_lowerRollerMotor.restoreFactoryDefaults();

    m_lowerEncoder.setPositionConversionFactor(.1);

    m_lowerEncoder.setVelocityConversionFactor(.1);

    m_lowerRollerMotor.setInverted(false);

    m_lowerRollerMotor.setIdleMode(IdleMode.kBrake);

    setLowerRollerBrakeOn(true);

    calibrateLowerPID();

    if (RobotBase.isSimulation()) {

      lowRollEncoderSim = new CANEncoderSim(m_lowerRollerMotor.getDeviceId(), false);
      lowRollerSim = new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
          DCMotor.getNeo550(1),
          4096);

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PROXVAL", rcs.getProximity());
    SmartDashboard.putNumber("LowRollOut", getLowerRoller());

  }

  @Override
  public void simulationPeriodic() {

    var vin = m_lowerRollerMotor.getAppliedOutput() * RobotController.getInputVoltage();

    lowRollerSim.setInputVoltage(vin);
    lowRollerSim.update(0.02);
    lowRollEncoderSim.setVelocity(lowRollerSim.getAngularVelocityRPM());

  }

  public boolean checkCAN() {

    lowerRollerMotorConnected = m_lowerRollerMotor.getFirmwareVersion() != -1;

    return lowerRollerMotorConnected;

  }

  public void reverseLowRoller() {

    runLowerAtVelocity(-500);

  }

  public double cargoAboveShoot() {

    return cargoAboveLowRoll.getAverageVoltage();
  }

  public boolean getCargoAtShoot() {

    return cargoSensor.get();
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

  public boolean getCargoAllianceMisMatch() {

    cargoIsBlue = getCargoIsBlue();

    cargoIsRed = getCargoIsRed();

    return getCargoAtShoot() && ((getAllianceBlue() && cargoIsRed)

        || (!getAllianceBlue() && cargoIsBlue));
  }

  public boolean getCargoRed() {

    return cargoIsRed;
  }

  public boolean getCargoBlue() {
    
    return cargoIsBlue;
  }

  public void releaseCargo() {

    double rpm = Pref.getPref("LowRollReleaseRPM");

    runLowerAtVelocity(rpm);

  }

  public void runLowerAtVelocity(double rpm) {

    lowerRequiredRPM = rpm;

    m_lowerPID.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
  }

  public boolean getLowerRollerAtSpeed() {

    return Math.abs(lowerRequiredRPM - getLowerRPM()) < (lowerRequiredRPM * .05);// getmps is -
  }

  public double getLowerRPM() {

    return Math.round(m_lowerEncoder.getVelocity() * 10) / 10;
  }

  public void runLowerRollerMotor(double speed) {
    m_lowerRollerMotor.set(speed);
  }

  public double getLowerRoller() {
    return m_lowerRollerMotor.getAppliedOutput();
  }

  public void stopLowerRoller() {
    lowerRequiredRPM = 0;
    m_lowerRollerMotor.set(0);
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

    m_lowerPID.setOutputRange(-1, 1, VELOCITY_SLOT);

    m_lowerRollerMotor.setClosedLoopRampRate(acc);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Cargo_transport");
    builder.addBooleanProperty("cargo_at_shoot", this::getCargoAtShoot, null);
    builder.addDoubleProperty("low_roll_out", this::getLowerRoller, null);
    builder.addDoubleProperty("low_roll_rpm", this::getLowerRPM, null);
    builder.addBooleanProperty("cargo_is_blue", this::getCargoBlue, null);
    builder.addBooleanProperty("cargo_is_red", this::getCargoRed, null);
    builder.addBooleanProperty("cargo_wrong_color", this::getCargoAllianceMisMatch, null);

  }
}
