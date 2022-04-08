package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Pref;
import frc.robot.Sim.CANEncoderSim;
import frc.robot.Sim.CANSparkMaxWithSim;

/**
 * 21:1 gearing
 * 
 * 222 to 18 gear to pinion
 * 
 * 21 revs of motor = 1 rev of pinion = 18/222 of 360 degrees = (18 * 360) /222
 * 
 * 1 rev of motor = (18 * 360)/(21 * 222) = 1.38996 degrees
 * 
 * neo 550 motor max rpm 11,000
 */

public class RevTurretSubsystem extends SubsystemBase {

    private static final double DEG_PER_MOTOR_REV = TurretConstants.TURRET_DEG_PER_MOTOR_REV;
    private static final int SMART_MOTION_SLOT = 1;
    public final int POSITION_SLOT = 2;
    // public final int VELOCITY_SLOT = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    public final CANSparkMaxWithSim m_motor; // NOPMD
    private final RelativeEncoder mEncoder;
    private CANEncoderSim mEncoderSim;
    private PIDController m_simpid;

    public final SparkMaxPIDController mPidController;
    public final PIDController m_turretLockController = new PIDController(.03, 0, 0);
    public SparkMaxLimitSwitch m_reverseLimit;
    public SparkMaxLimitSwitch m_forwardLimit;

    public double targetAngle;
    private double inPositionBandwidth = 2;
    public double targetHorizontalOffset;
    public double pset, iset, dset, ffset, izset, maxAccset, maxVelset;
    public double psetv, isetv, dsetv, ffsetv, izsetv;
    public double lpset, liset, ldset, lizset;

    public boolean validTargetSeen;
    public double adjustedCameraError;

    public boolean tuneOn = false;
    public boolean lastTuneOn;
    public boolean tuneOnv = false;
    public boolean lastTuneOnv;

    public boolean lockTuneOn = false;
    public boolean lastLockTuneOn;

    public boolean turretMotorConnected;

    public double lockPIDOut;
    public boolean visionOnTarget;
    public double adjustMeters = .15;// 6"
    private double maxAdjustMeters = .5;
    private double minAdjustMeters = -.5;
    public double driverAdjustAngle;
    public double driverAdjustDistance;
    public NetworkTableEntry setupHorOffset;
    public double driverHorizontalOffsetDegrees;
    public double driverHorizontalOffsetMeters;
    public double turretSetupOffset;

    public boolean useSetupHorOffset;
    public double testHorOffset;
    public double positionError;
    public double correctedEndpoint;
    public double visionErrorDifference;
    public double programRunning;// 1-hold 2 position 3 vision

    public double turretVisionTolerance = 1;
    public double testLockFromThrottle;
    public boolean testLock;

    public boolean turretUseVision;
    public double turretOffsetAdder;
    public double turretOffsetChange;
    public double presetPosition;

    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    LinearSystem<N2, N1, N1> m_turretPosition = LinearSystemId
            .identifyPositionSystem(TurretConstants.kVVoltSecondsPerRotation, TurretConstants.kA);
    LinearSystemSim<N2, N1, N1> m_turretPositionSim = new LinearSystemSim<>(m_turretPosition);

    public RevTurretSubsystem() {
        m_motor = new CANSparkMaxWithSim(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(5);
        m_motor.setClosedLoopRampRate(2);

        mEncoder.setPosition(0);
        aimCenter();
        setFF_MaxOuts();
        tunePosGains();
        getPosGains();
        setTurretLockGains();
        getLockGains();
        m_turretLockController.setTolerance(.5);
        setSoftwareLimits();

        if (RobotBase.isReal()) {
            mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);
            mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);

        }

        if (RobotBase.isSimulation()) {

            mEncoderSim = new CANEncoderSim(m_motor.getDeviceId(), false);

            m_simpid = new PIDController(.01, 0, 0);

        }

        m_reverseLimit = m_motor.getReverseLimitSwitch(Type.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(true);

        m_forwardLimit = m_motor.getForwardLimitSwitch(Type.kNormallyClosed);
        m_forwardLimit.enableLimitSwitch(true);



    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        tuneOn = Pref.getPref("tURTune") != 0.;

        checkTune();

        if (DriverStation.isDisabled())
            targetAngle = getAngle();

        if (useSetupHorOffset) {
            testHorOffset = 0;// setupHorOffset.getDouble(0);
        } else {
            testHorOffset = 0;
        }
      
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our tilt is doing
        // First, we set our "inputs" (voltages)
        m_turretPositionSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_turretPositionSim.update(0.020);
        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        mEncoderSim.setPosition(m_turretPositionSim.getOutput(0));
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretPositionSim.getCurrentDrawAmps()));


    }

    public boolean checkCAN() {
        return turretMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void moveManually(double speed) {

        targetAngle = getAngle();

        m_motor.set(speed);

    }

    public void goToPosition(double degrees) {

        double pidout = 0;

        if (RobotBase.isReal()) {

            mPidController.setReference(degrees, ControlType.kPosition, POSITION_SLOT);

        }

        else {

            pidout = m_simpid.calculate(getAngle(), degrees);

            if (pidout > .75)
                pidout = .75;
            if (pidout < -.75)
                pidout = -.75;

            m_motor.set(pidout);
        }

    }

    public double getTargetHorOffset() {
        
        return targetHorizontalOffset;
    }

    public void goToPositionMotionMagic(double degrees) {

        if (RobotBase.isReal())

        mPidController.setReference(degrees, ControlType.kSmartMotion, SMART_MOTION_SLOT);

    else

        goToPosition(degrees);


    }

    public void lockTurretToVision(double cameraError) {

        lockPIDOut = m_turretLockController.calculate(cameraError, 0);

        runAtVelocity(lockPIDOut);

        targetAngle = getAngle();
    }

    public void lockTurretToThrottle(double throttleError) {

        lockPIDOut = m_turretLockController.calculate(throttleError, 0);

        runAtVelocity(lockPIDOut);
        targetAngle = getAngle();
    }

    public double getLockPositionError() {
        return m_turretLockController.getPositionError();
    }

    public boolean getLockAtTarget() {
        return m_turretLockController.atSetpoint();
    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
        mPidController.setIAccum(0);
        targetAngle = angle;
    }

    public double getMotorOut() {
        return m_motor.getAppliedOutput();
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .05;
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getSpeed() {
        return mEncoder.getVelocity();
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                (float) TurretConstants.TURRET_MAX_ANGLE);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                (float) TurretConstants.TURRET_MIN_ANGLE);
        enableSofLimits(true);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSofLimits(boolean on) {
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onPlusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitFwd);
    }

    public void aimFurtherLeft() {
        if (driverHorizontalOffsetMeters > minAdjustMeters) {
            driverHorizontalOffsetDegrees -= driverAdjustAngle;
            driverHorizontalOffsetMeters -= adjustMeters;
        }
    }

    public void aimFurtherRight() {
        if (driverHorizontalOffsetMeters < maxAdjustMeters) {
            driverHorizontalOffsetDegrees += driverAdjustAngle;
            driverHorizontalOffsetMeters += adjustMeters;
        }
    }

    public void aimCenter() {
        driverHorizontalOffsetMeters = 0;
        driverHorizontalOffsetDegrees = 0;
    }

    public boolean isAtHeight(double angle, double allowableError) {
        return Math.abs(angle - getAngle()) < allowableError;
    }

    public boolean atTargetAngle() {

        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    public double getAngle() {

        if (RobotBase.isReal())

            return mEncoder.getPosition();
        else

            return mEncoder.getPosition() * DEG_PER_MOTOR_REV;

    }

    public void stop() {
        m_motor.set(0);
    }

    public void runAtVelocity(double speed) {
        
        targetAngle = getAngle();
        mPidController.setReference(speed, ControlType.kVelocity, SMART_MOTION_SLOT);
    }

    public double getIaccum() {
        return mPidController.getIAccum();
    }

    public void calibratePID(final double p, final double i, final double d, final double kIz, int slotNumber) {
        mPidController.setFF(kFF, SMART_MOTION_SLOT);
        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = mPidController.getP(slotNumber);

        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = mPidController.getI(slotNumber);
        }

        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = mPidController.getD(slotNumber);
        }

        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIz = mPidController.getIZone(slotNumber);
        }

        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mPidController.setSmartMotionMaxVelocity(maxVel, slotNumber);
                lastmaxVel = maxVel;
            }

            if (lastallowedErr != allowedErr) {
                mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);
                lastallowedErr = allowedErr;
            }
        }
    }

    private void setFF_MaxOuts() {
        kMinOutput = -.5;
        kMaxOutput = .5;
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        mPidController.setFF(kFF, SMART_MOTION_SLOT);

    }

    private void tunePosGains() {
        kFF = 0;// Pref.getPref("tURKff");// 10,000/60 rps* 1.39 = 231. and 1/237 = .004
        double p = Pref.getPref("tURKp");
        double i = Pref.getPref("tURKi");
        double d = Pref.getPref("tURKd");
        double iz = Pref.getPref("tURKiz");
        kMinOutput = -.5;
        kMaxOutput = .5;
        mPidController.setOutputRange(kMinOutput, kMaxOutput, POSITION_SLOT);
        allowedErr = .1;
        calibratePID(p, i, d, iz, POSITION_SLOT);

    }

    private void setTurretLockGains() {

        m_turretLockController.setP(Pref.getPref("TuLkP"));
        m_turretLockController.setI(Pref.getPref("TuLkI"));
        m_turretLockController.setD(Pref.getPref("TuLkD"));
        lizset = Pref.getPref("TuLkIZ");
        m_turretLockController.setIntegratorRange(-lizset, lizset);
        m_turretLockController.setTolerance(.5);
    }

    private void checkTune() {

        tuneOn = Pref.getPref("tURTune") == 1. && turretMotorConnected;

        if (tuneOn && !lastTuneOn) {
            tunePosGains();
            getPosGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

        // lock controller

        lockTuneOn = Pref.getPref("tuLTune") != 0.;

        if (lockTuneOn && !lastLockTuneOn) {

            setTurretLockGains();
            lastLockTuneOn = true;
            getLockGains();
        }

        if (lastLockTuneOn)
            lastLockTuneOn = lockTuneOn;

    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void getPosGains() {
        ffset = mPidController.getFF(POSITION_SLOT);
        pset = mPidController.getP(POSITION_SLOT);
        iset = mPidController.getI(POSITION_SLOT);
        dset = mPidController.getD(POSITION_SLOT);
        izset = mPidController.getIZone(POSITION_SLOT);

    }

    public void getLockGains() {
        lpset = m_turretLockController.getP();
        liset = m_turretLockController.getI();
        ldset = m_turretLockController.getD();

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Turret");
        builder.addBooleanProperty("turret_rev_switch", this::onMinusHardwareLimit, null);
        builder.addBooleanProperty("turret_fwd_switch", this::onPlusHardwareLimit, null);
        builder.addBooleanProperty("soft_lim_en", this::getSoftwareLimitsEnabled, null);
        builder.addDoubleProperty("angle_degrees", this::getAngle, null);
        builder.addDoubleProperty("motor_out", this::getMotorOut, null);
        builder.addBooleanProperty("at_plus_soft_lim", this::onPlusSoftwareLimit, null);
        builder.addBooleanProperty("at_min_soft_lim", this::onMinusSoftwareLimit, null);
        builder.addBooleanProperty("at_target", this::atTargetAngle, null);

    }

}
