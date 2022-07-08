package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Pref;
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
    // private static final int SMART_MOTION_SLOT = 1;
    // public final int VELOCITY_SLOT = 2;
    public final int VELOCITY_SLOT = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    public final CANSparkMaxWithSim m_motor; // NOPMD
    private final RelativeEncoder mEncoder;
    public PIDController mPosController;

    public final SparkMaxPIDController mVelController;
    public final PIDController mLockController = new PIDController(.03, 0, 0);
    public SparkMaxLimitSwitch m_reverseLimit;
    public SparkMaxLimitSwitch m_forwardLimit;
    WPI_CANCoder _CANCoder = new WPI_CANCoder(0, "TurretCANCoder");

    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();

    private double inPositionBandwidth = 2;
    public double targetHorizontalOffset;
    public double psetv, isetv, dsetv, ffsetv, izsetv, maxAccset, maxVelset;
    public double psetp, isetp, dsetp, izsetp;
    public double psetl, isetl, dsetl, izsetl;

    public boolean validTargetSeen;

    public boolean tuneOnv = false;
    public boolean lastTuneOnv;

    public boolean lockTuneOn = false;
    public boolean lastLockTuneOn;

    private boolean posTuneOn;
    private boolean lastPosTuneOn;

    public boolean turretMotorConnected;

    public double lockPIDOut;
    public boolean visionOnTarget;
    public double positionError;
    public double correctedEndpoint;
    public double visionErrorDifference;
    public double programRunning;// 1-hold 2 position 3 vision

    public double turretVisionTolerance = 1;
    public double testLockFromThrottle;
    public boolean testLock;

    public double turretOffsetAdder;
    public double turretOffsetChange;
    public double presetPosition;
    public double targetAngle;

    private CTRECanCoder tuCanCoder = new CTRECanCoder(CANConstants.TURRET_CANCODER);
    private double m_startUpAbsoluteValue;
    private double startUpAngleFromZeroValue;

    public RevTurretSubsystem() {
        m_motor = new CANSparkMaxWithSim(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mVelController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(5);
        m_motor.setClosedLoopRampRate(1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        m_motor.setSmartCurrentLimit(20);
        mPosController = new PIDController(.01, 0, 0);
        mEncoder.setPosition(0);

        if (RobotBase.isReal()) {
            // setFF_MaxOuts();
            setVelGains();
            getVelGains();
            setPosGains();
            setLockGains();
            mLockController.setTolerance(.25);

        }
        setSoftwareLimits();

        mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);

        mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);

        if (RobotBase.isSimulation()) {

        }

        m_reverseLimit = m_motor.getReverseLimitSwitch(Type.kNormallyClosed);

        m_reverseLimit.enableLimitSwitch(true);

        m_forwardLimit = m_motor.getForwardLimitSwitch(Type.kNormallyClosed);

        m_forwardLimit.enableLimitSwitch(true);

        m_startUpAbsoluteValue = tuCanCoder.getAbsValue();

        startUpAngleFromZeroValue = m_startUpAbsoluteValue - TurretConstants.ABSZEROVALUE;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // SmartDashboard.putNumber("Tple", getLockPositionError());
        // SmartDashboard.putBoolean("tlat", getLockAtTarget());
        // SmartDashboard.putNumber("tlmo", getMotorOut());

        // SmartDashboard.putData("LockPID",mLockController);

        if (RobotBase.isReal())

            checkTune();

        if (DriverStation.isDisabled()) {
            targetAngle = getAngle();
        }

    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        // m_turretSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
        // SmartDashboard.putNumber("TUSIMI", m_motor.get() *
        // RobotController.getBatteryVoltage());
        // // Next, we update it. The standard loop time is 20ms.
        // m_turretSim.update(0.020);
        // SmartDashboard.putNumber("TUSIMO", m_turretSim.getOutput(0));
        // // Finally, we set our simulated encoder's readings and simulated battery
        // // voltage
        // mEncoderSim.setPosition(m_turretSim.getOutput(0));
        // // SimBattery estimates loaded battery voltages
        // RoboRioSim.setVInVoltage(
        // BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretSim.getCurrentDrawAmps()));

    }

    public boolean checkCAN() {
        return turretMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void moveManually(double speed) {

        if (RobotBase.isReal()) {
            m_motor.set(speed);
        }

        else {

            m_motor.setVoltage(speed * 12);
        }

        targetAngle = getAngle();

    }

    public void goToPosition(double degrees) {

        double pidout = 0;

        pidout = mPosController.calculate(getAngle(), degrees);
        // SmartDashboard.putNumber("TUPERR", mPosController.getPositionError());
        // SmartDashboard.putNumber("PosPGain", mPosController.getP());

        // SmartDashboard.putNumber("TUPIDOUT", -pidout);

        moveAtVelocity(-pidout * maxVel);
    }

    public double getTargetHorOffset() {

        return targetHorizontalOffset;
    }

    public void lockTurretToVision(double cameraError) {
        double maxAllowedCameraError = 10;
        boolean negError = cameraError < 0;
        // if (Math.abs(cameraError) > maxAllowedCameraError) {
        //     cameraError = maxAllowedCameraError;
        //     if (negError)
        //         cameraError = -cameraError;

        // }

        lockPIDOut = mLockController.calculate(cameraError, 0);

        double maxAllowedOut = .5;

        if (lockPIDOut > maxAllowedOut)
            lockPIDOut = maxAllowedOut;
        if (lockPIDOut < -maxAllowedOut)
            lockPIDOut = -maxAllowedOut;

        // else

        //     lockPIDOut = 0;

        SmartDashboard.putNumber("tulLockMove", lockPIDOut * maxVel);

        moveAtVelocity(lockPIDOut * maxVel);

        targetAngle = getAngle();

    }

    public void lockTurretToThrottle(double throttleError) {

        lockPIDOut = mLockController.calculate(throttleError, 0);

        moveAtVelocity(lockPIDOut);

        targetAngle = getAngle();
    }

    public double getLockPositionError() {
        return mLockController.getPositionError();
    }

    public boolean getLockAtTarget() {
        return mLockController.atSetpoint();
    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
        // if (RobotBase.isSimulation())
        // mEncoderSim.setPosition(angle);

        targetAngle = angle;
    }

    public double getMotorOut() {
        return m_motor.get();
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

            return mEncoder.getPosition();

    }

    public void stop() {

        m_motor.set(0);
    }

    public void moveAtVelocity(double degPerSec) {

        if (RobotBase.isReal()) {

            mVelController.setReference(-degPerSec, ControlType.kVelocity, VELOCITY_SLOT);

            SmartDashboard.putNumber("TURVEl", -degPerSec);

            SmartDashboard.putNumber("TuMAOut", m_motor.getAppliedOutput());

        }

        else {

            moveManually(degPerSec / maxVel);
        }

    }

    public double getIaccum() {
        return mVelController.getIAccum();
    }

    public void calibratePID(final double p, final double i, final double d, final double kIz, int slotNumber) {
        mVelController.setFF(kFF, VELOCITY_SLOT);
        if (p != lastkP) {
            mVelController.setP(p, slotNumber);
            lastkP = mVelController.getP(slotNumber);

        }
        if (i != lastkI) {
            mVelController.setI(i, slotNumber);
            lastkI = mVelController.getI(slotNumber);
        }

        if (d != lastkD) {
            mVelController.setD(d, slotNumber);
            lastkD = mVelController.getD(slotNumber);
        }

        if (kIz != lastkIz) {
            mVelController.setIZone(kIz, slotNumber);
            lastkIz = mVelController.getIZone(slotNumber);
        }

    }

    private void setFF_MaxOuts() {
        kFF = 0;// not used inside position loop
        kMinOutput = -.75;
        kMaxOutput = .75;
        mVelController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);
        mVelController.setFF(kFF, VELOCITY_SLOT);

    }

    private void setVelGains() {
        // kFF = Pref.getPref("tuVKff");// 9,000/60 rps = 150rps * 1.39 degperrev = 210
        // and 1/210 = .004
        double p = Pref.getPref("tuVKp");
        double i = Pref.getPref("tuVKi");
        double d = Pref.getPref("tuVKd");
        double iz = Pref.getPref("tuVKiz");
        kMinOutput = -.75;
        kMaxOutput = .75;
        mVelController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);
        maxVel = 100;// Pref.getPref("tuVMaxV");// deg per sec 150rps *1.39 = 208 deg /sec
        maxAcc = Pref.getPref("tuVMaxA");// deg per sec per sec
        allowedErr = .1;
        calibratePID(p, i, d, iz, VELOCITY_SLOT);

    }

    private void setPosGains() {

        mPosController.setP(Pref.getPref("tuPkp"));
        mPosController.setI(Pref.getPref("tuPki"));
        mPosController.setD(Pref.getPref("tuPkd"));
        izsetp = Pref.getPref("tuPkiz");
        mPosController.setIntegratorRange(-izsetp, izsetp);
        mPosController.setTolerance(.5);

        SmartDashboard.putNumber("PREFTUPKP", Pref.getPref("tuPkp"));
    }

    private void setLockGains() {

        mLockController.setP(Pref.getPref("tuLkp"));

        SmartDashboard.putNumber("TulpPrkp", Pref.getPref("tuLkp"));

        SmartDashboard.putNumber("TULRDP", mLockController.getP());
        mLockController.setI(Pref.getPref("tuLki"));
        mLockController.setD(Pref.getPref("tuLkd"));
        izsetl = Pref.getPref("tuLkiz");
        mLockController.setIntegratorRange(-izsetl, izsetl);
        mLockController.setTolerance(.5);
    }

    private void checkTune() {

        tuneOnv = Pref.getPref("tuVTune") == 1. && turretMotorConnected;

        if (tuneOnv && !lastTuneOnv) {
            setVelGains();
            getVelGains();
            lastTuneOnv = true;
        }

        if (lastTuneOnv)
            lastTuneOnv = tuneOnv;

        // lock controller

        lockTuneOn = Pref.getPref("tuLTune") != 0.;

        if (lockTuneOn && !lastLockTuneOn) {
            setLockGains();
            lastLockTuneOn = true;
        }

        if (lastLockTuneOn)
            lastLockTuneOn = lockTuneOn;

        // posn controller

        posTuneOn = Pref.getPref("tuPTune") != 0.;

        if (posTuneOn && !lastPosTuneOn) {
            setPosGains();
            lastPosTuneOn = true;
        }

        if (lastPosTuneOn)
            lastPosTuneOn = posTuneOn;

    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void getVelGains() {
        ffsetv = mVelController.getFF(VELOCITY_SLOT);
        psetv = mVelController.getP(VELOCITY_SLOT);
        isetv = mVelController.getI(VELOCITY_SLOT);
        dsetv = mVelController.getD(VELOCITY_SLOT);
        izsetv = mVelController.getIZone(VELOCITY_SLOT);

    }

}
