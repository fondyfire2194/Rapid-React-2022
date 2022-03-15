package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.Pref;

public class RevTiltSubsystem extends SubsystemBase {

    // public final int VELOCITY_SLOT = 0;
    public final int SMART_MOTION_SLOT = 1;
    public final int POSITION_SLOT = 2;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    // public double kPv, kIv, kDv, kIzv, kFFv, kMaxOutputv, kMinOutputv, maxRPMv,
    // maxVelv, minVelv, maxAccv, allowedErrv;
    // public double lastkPv, lastkIv, lastkDv, lastkIzv, lastkFFv, lastkMaxOutputv,
    // lastkMinOutputv, lastmaxRPMv,
    // lastmaxVelv, lastmv, lastmaxAccv, lastallowedErrv;

    public final CANSparkMax m_motor; // NOPMD
    private final RelativeEncoder mEncoder;
    public final SparkMaxPIDController mPidController;
    // public final PIDController tiltLockController = new PIDController(.032,
    // 0.001, 0);
    public SparkMaxLimitSwitch m_reverseLimit;
    public boolean positionResetDone;
    public double targetAngle;
    private double inPositionBandwidth = 1;
    public double targetVerticalOffset;
    public double driverVerticalOffsetDegrees;
    public double driverVerticalOffsetMeters;

    public boolean validTargetSeen;
    public double adjustedVerticalError;

    public final double degreesPerRev = TiltConstants.tiltDegreesPerRev;// degrees per motor turn
    public final double tiltMinAngle = TiltConstants.TILT_MIN_ANGLE;

    public double pset, iset, dset, ffset, izset, maxAccset, maxVelset;
    public double psetv, isetv, dsetv, ffsetv, izsetv;
    public double lpset, liset, ldset, lizset;

    public boolean tuneOn = false;
    public boolean lastTuneOn;

    public boolean lockTuneOn = false;
    public boolean lastLockTuneOn;

    public boolean tiltMotorConnected;
    private double maxAdjustMeters = .5;
    private double minAdjustMeters = -.5;
    // public double motorEndpointDegrees;
    public int faultSeen;
    public double lockPIDOut;
    public double lockPIDOutVolts;
    public boolean visionOnTarget;
    public double driverAdjustAngle;
    public double driverAdjustDistance;
    public double adjustMeters = .16;// 6"

    public double testLockFromThrottle;

    public double tiltSetupOffset;

    public double testVerticalOffset;

    public double positionError;
    public double correctedEndpoint;
    public double visionErrorDifference;
    public double highTolerance;
    public double lowTolerance;
    public double cameraAngle;

    public double programRunning;// 1-hold 2 position 3 vision

    public boolean testLock;
    public boolean tiltUseVision;;
    public double tiltOffsetAdder;
    public double tiltOffsetChange;
    public double cameraCalculatedTiltPosition;
    public double presetPosition;
    public NetworkTableEntry tiltTarget;

    /** 
     * 
     * 
     * 
     * 
     * 
     */

    public RevTiltSubsystem() {
        m_motor = new CANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(5);
        aimCenter();
        mEncoder.setPosition(0);
        targetAngle = 0;

        mEncoder.setPositionConversionFactor(degreesPerRev);
        mEncoder.setVelocityConversionFactor(degreesPerRev / 60);

        m_motor.setSmartCurrentLimit(3);

        setFF_MaxOuts();
        tunePosGains();
        // tuneVelGains();

        getPosGains();
        // getVelGains();

        resetAngle();
        m_motor.setIdleMode(IdleMode.kBrake);

        m_reverseLimit = m_motor.getReverseLimitSwitch(Type.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(true);

        if (RobotBase.isReal() && m_reverseLimit.isPressed()) {
            resetAngle();
        }

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNeo550(1));

        }

        setSoftwareLimits();
        SmartDashboard.putNumber("TIdegPerRev", degreesPerRev);

        tiltTarget = Shuffleboard.getTab("SetupTilt")
                .add("TargetDegrees", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 3).withSize(2, 1)
                .withProperties(Map.of("min", 0, "max", 14))
                .getEntry();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkTune();

        if (RobotBase.isReal() && DriverStation.isDisabled())
            targetAngle = getAngle();

        if (faultSeen != 0)
            faultSeen = getFaults();

        SmartDashboard.putNumber("TIMTRGETT", m_motor.getAppliedOutput());

    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();

    }

    public boolean checkCAN() {
        return tiltMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public double getIaccum() {
        return mPidController.getIAccum();
    }

    public void close() {
        m_motor.close();
    }

    public void runAtVelocity(double speed) {
        targetAngle = getAngle();
        SmartDashboard.putNumber("TILSPEED", speed);
        mPidController.setReference(speed, ControlType.kVelocity, SMART_MOTION_SLOT);
    }

    public void moveManually(double speed) {
        targetAngle = getAngle();
        if (RobotBase.isReal())
            m_motor.set(speed);
        else
            mPidController.setReference(speed * 12, ControlType.kVoltage, POSITION_SLOT);

    }

    public void goToPosition(double degrees) {
        if (RobotBase.isReal())
            mPidController.setReference(degrees, ControlType.kPosition, POSITION_SLOT);

    }

    public void goToPositionMotionMagic(double degrees) {

        // convert angle to motor turns

        mPidController.setReference(degrees, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public void resetAngle() {
        mEncoder.setPosition(0);
        targetAngle = 0;
        mPidController.setIAccum(0);
    }

    public boolean isAtHeight(double inches, double allowableError) {
        return Math.abs(inches - getAngle()) < allowableError;
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    public double getMotorDegrees() {
        return mEncoder.getPosition();
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .05;
    }

    public double getAngle() {
        return getMotorDegrees();
    }

    public double getCameraAngle() {
        return getAngle();
    }

    public double getOut() {
        return m_motor.getAppliedOutput();
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getSpeed() {
        return mEncoder.getVelocity();
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onMinusHardwarLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public void stop() {
        if (RobotBase.isReal())
            m_motor.set(0);
        else
            m_motor.setVoltage(0);
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) TiltConstants.TILT_MIN_ANGLE);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) TiltConstants.TILT_MAX_ANGLE);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSoftLimits(boolean on) {
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public void clearFaults() {
        m_motor.clearFaults();
        faultSeen = 0;
    }

    public double getVerticalTargetOffset() {
        return targetVerticalOffset;
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void aimLower() {

        if (driverVerticalOffsetMeters > minAdjustMeters) {
            driverVerticalOffsetDegrees -= driverAdjustAngle;
            driverVerticalOffsetMeters -= adjustMeters;
        }
    }

    public void aimHigher() {

        if (driverVerticalOffsetMeters < maxAdjustMeters) {
            driverVerticalOffsetDegrees += driverAdjustAngle;
            driverVerticalOffsetMeters += adjustMeters;
        }
    }

    public void aimCenter() {
        driverVerticalOffsetDegrees = 0;
        driverVerticalOffsetMeters = 0;
    }

    public void calibratePID(final double p, final double i, final double d, final double kIz, final double allE,
            int slotNumber) {
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
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }

    }

    private void setFF_MaxOuts() {

        kFF = 0;// 10,000 rpm = (10000/60) * .03645 = 6.0 1/6.0 = .167
        kMinOutput = -.5;
        kMaxOutput = .6;

        mPidController.setFF(kFF, POSITION_SLOT);

        mPidController.setOutputRange(kMinOutput, kMaxOutput, POSITION_SLOT);

    }

    private void tunePosGains() {

        double p = Pref.getPref("tIKp");
        double i = Pref.getPref("tIKi");
        double d = Pref.getPref("tIKd");
        double iz = Pref.getPref("tIKiz");
        allowedErr = .01;
        kMinOutput = -.5;
        kMaxOutput = .5;
        mPidController.setOutputRange(kMinOutput, kMaxOutput, POSITION_SLOT);

        calibratePID(p, i, d, iz, allowedErr, POSITION_SLOT);

    }

    private void checkTune() {

        tuneOn = Pref.getPref("tITune") == 1. && tiltMotorConnected;

        if (tuneOn && !lastTuneOn) {

            tunePosGains();
            getPosGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

    }

    public void getPosGains() {
        ffset = mPidController.getFF(POSITION_SLOT);
        pset = mPidController.getP(POSITION_SLOT);
        iset = mPidController.getI(POSITION_SLOT);
        dset = mPidController.getD(POSITION_SLOT);
        izset = mPidController.getIZone(POSITION_SLOT);
        maxAccset = mPidController.getSmartMotionMaxAccel(POSITION_SLOT);
        maxVelset = mPidController.getSmartMotionMaxVelocity(POSITION_SLOT);

    }

}
