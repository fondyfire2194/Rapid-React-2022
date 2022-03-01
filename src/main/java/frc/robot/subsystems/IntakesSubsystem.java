// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SolenoidConstants;
import frc.robot.Vision.LimeLight;

public class IntakesSubsystem extends SubsystemBase {
  /** Creates a new Intakes. */

  public boolean useFrontIntake;

  private final WPI_TalonSRX m_rearIntakeMotor = new WPI_TalonSRX(CANConstants.REAR_INTAKE_MOTOR);

  public final DoubleSolenoid m_rearIntakeArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      SolenoidConstants.REAR_INTAKE_1, SolenoidConstants.REAR_INTAKE_2);

  private final WPI_TalonSRX m_frontIntakeMotor = new WPI_TalonSRX(CANConstants.FRONT_INTAKE_MOTOR);

  public final DoubleSolenoid m_frontIntakeArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      SolenoidConstants.FRONT_INTAKE_1, SolenoidConstants.FRONT_INTAKE_2);

  public boolean frontIntakeMotorConnected;

  public boolean rearIntakeMotorConnected;

  public int redCargoPipeline = 0;
  public int blueCargoPipeline = 1;
  public int redLauchPadPipeline = 2;
  public int blueLaunchPadPipeline = 3;

  public int activeCargoPipeline = redCargoPipeline;
  
  public int activeLaunchPadPipeline = redLauchPadPipeline;

  private AnalogInput frontIntakeCargoDetect;

  private AnalogInput rearIntakeCargoDetect;// next to be released for shooting

  private double cargoDetectedVolts = 2.75;

  public LimeLight frontll = new LimeLight("limelightfront");

  public LimeLight rearll = new LimeLight("limelightrear");

  public IntakesSubsystem() {

    m_rearIntakeMotor.configFactoryDefault();
    m_rearIntakeMotor.setNeutralMode(NeutralMode.Brake);
    m_frontIntakeMotor.configFactoryDefault();
    m_frontIntakeMotor.setNeutralMode(NeutralMode.Brake);

    raiseRearArm();
    raiseFrontArm();

    frontIntakeCargoDetect = new AnalogInput(1);

    rearIntakeCargoDetect = new AnalogInput(2);

    frontll.setPipeline(8);// no leds

    rearll.setPipeline(8);
  }

  public void toggleActiveIntake() {

    useFrontIntake = !useFrontIntake;
  }

  public boolean getActiveIntake() {
    return useFrontIntake;
  }

  public void setFrontActive() {
    useFrontIntake = true;
  }

  public void setRearActive() {
    useFrontIntake = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getCargoAtFront() {
    return frontIntakeCargoDetect.getVoltage() > cargoDetectedVolts;

  }

  public boolean getCargoAtRear() {
    return rearIntakeCargoDetect.getVoltage() > cargoDetectedVolts;

  }

  public boolean checkFrontCAN() {
    return frontIntakeMotorConnected = m_frontIntakeMotor.getFirmwareVersion() != -1;

  }

  public void runFrontIntakeMotor(double speed) {
    m_frontIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopFrontIntakeMotor() {
    m_frontIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getFrontMotor() {
    return m_frontIntakeMotor.getMotorOutputPercent();
  }

  public double getFrontMotorAmps() {
    return m_frontIntakeMotor.getStatorCurrent();
  }

  public void setFrontBrakeOn(boolean on) {
    if (on) {
      m_frontIntakeMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_frontIntakeMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void raiseFrontArm() {
    m_frontIntakeArm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lowerFrontArm() {
    m_frontIntakeArm.set(DoubleSolenoid.Value.kForward);
  }

  public boolean getFrontArmLowered() {
    return m_frontIntakeArm.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getFrontArmRaised() {
    return m_frontIntakeArm.get() == DoubleSolenoid.Value.kReverse;
  }

  public void frontArmSolenoidOff() {
    m_frontIntakeArm.set(DoubleSolenoid.Value.kOff);
  }

  public boolean getFrontArmOff() {
    return m_frontIntakeArm.get() == DoubleSolenoid.Value.kOff;
  }

  /// Rear intake starts here
  public boolean checkRearCAN() {
    return rearIntakeMotorConnected = m_rearIntakeMotor.getFirmwareVersion() != -1;

  }

  public void runRearIntakeMotor(double speed) {
    m_rearIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopRearIntakeMotor() {
    m_rearIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getRearMotor() {
    return m_rearIntakeMotor.getMotorOutputPercent();
  }

  public double getRearMotorAmps() {
    return m_rearIntakeMotor.getStatorCurrent();
  }

  public void setRearBrakeOn(boolean on) {
    if (on) {
      m_rearIntakeMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_rearIntakeMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void raiseRearArm() {
    m_rearIntakeArm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lowerRearArm() {
    m_rearIntakeArm.set(DoubleSolenoid.Value.kForward);
  }

  public boolean getRearArmLowered() {
    return m_rearIntakeArm.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getRearArmRaised() {
    return m_rearIntakeArm.get() == DoubleSolenoid.Value.kReverse;
  }

  public void rearArmSolenoidOff() {
    m_rearIntakeArm.set(DoubleSolenoid.Value.kOff);
  }

  public boolean getRearArmOff() {
    return m_rearIntakeArm.get() == DoubleSolenoid.Value.kOff;
  }

  public boolean getActiveArmRaised() {
    if (useFrontIntake) {
      return getFrontArmRaised();
    } else
      return getRearArmRaised();
  }

  public boolean getActiveArmLowered() {
    if (useFrontIntake) {
      return getFrontArmLowered();
    } else
      return getRearArmLowered();
  }

  public double getActiveMotorAmps() {
    if (useFrontIntake) {
      return getFrontMotorAmps();
    } else
      return getRearMotorAmps();
  }

  public double getActiveMotor() {
    if (useFrontIntake) {
      return getFrontMotor();
    } else
      return getRearMotor();
  }

  public void setActivePipeline(int pipelineNumber) {
    frontll.setPipeline(pipelineNumber);
    rearll.setPipeline(pipelineNumber);
  }

  public double getActiveTargetYaw() {

    double temp = 0;

    if (useFrontIntake) {

      if (frontll.getIsTargetFound()) {

        temp = frontll.getdegVerticalToTarget();

      }
    }

    else {

      if (rearll.getIsTargetFound()) {

        temp = rearll.getdegVerticalToTarget();

      }
    }
    return temp;
  }

  public double getActiveTargetDist() {

    double temp = 0;

    if (useFrontIntake) {

      if (frontll.getIsTargetFound()) {

        temp = frontll.getdegRotationToTarget();

      }
    }

    else {

      if (rearll.getIsTargetFound()) {

        temp = rearll.getdegRotationToTarget();

      }
    }
    return temp;
  }

  
}