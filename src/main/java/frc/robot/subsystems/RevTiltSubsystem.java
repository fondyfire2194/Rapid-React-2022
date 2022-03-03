package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

    // public double kPv, kIv, kDv, kIzv, kFFv, kMaxOutputv, kMinOutputv, maxRPMv, maxVelv, minVelv, maxAccv, allowedErrv;
    // public double lastkPv, lastkIv, lastkDv, lastkIzv, lastkFFv, lastkMaxOutputv, lastkMinOutputv, lastmaxRPMv,
    //         lastmaxVelv, lastmv, lastmaxAccv, lastallowedErrv;

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
    private final static double pivotDistance = 10.5;// inches

    private final double[] pinDistances = { 2.1, 3.0842519685, 4.068503937, 5.0527559055, 6.037007874, 7.0212598425,
            8.005511811 };

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
    public boolean endTiltFile;

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
    public double cameraCalculatedTiltOffset;

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

        setFF_MaxOuts();
        tuneMMGains();
        // tuneVelGains();

        getMMGains();
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

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("degPerRev", degreesPerRev);
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

    public void goToPosition(double motorTurns) {
        if (RobotBase.isReal())
            mPidController.setReference(motorTurns, ControlType.kPosition);

    }

    public void positionTilt(double motorDegrees, int slotNumber) {
        mPidController.setReference(motorDegrees, ControlType.kPosition, slotNumber);

    }

    public void goToPositionMotionMagic(double motorDegrees) {

        // convert angle to motor turns

        mPidController.setReference(motorDegrees, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    // public void lockTiltToVision(double cameraError) {
    // lockPIDOut = tiltLockController.calculate(cameraError, 0);

    // runAtVelocity(lockPIDOut);

    // targetAngle = getAngle();
    // }

    // public void lockTiltToThrottle(double throttleError) {
    // SmartDashboard.putNumber("TTHERR", throttleError);
    // lockPIDOut = tiltLockController.calculate(throttleError, 0);
    // SmartDashboard.putNumber("PIDL OUT", lockPIDOut);
    // SmartDashboard.putNumber("PIDL Err", tiltLockController.getPositionError());
    // runAtVelocity(lockPIDOut);
    // targetAngle = getAngle();
    // }

    // public double getLockControllerOutput(double cameraError) {
    // return tiltLockController.calculate(cameraError, 0);
    // }

    // public void lockWithLockController(double cameraError) {
    // m_motor.set(tiltLockController.calculate(cameraError, 0));
    // }

    // public double getLockPositionError() {
    // return tiltLockController.getPositionError();
    // }

    // public boolean getLockAtTarget() {
    // return tiltLockController.atSetpoint();
    // }

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

    /**
     * The angle should be governed by angle =(2*(ASIN(distance between
     * pins/2)/10.5)) this will be in radians.
     * 
     * 
     * @return
     */
    // public double calculateTiltAngle() {
    // // separate angle into integer and remainder

    // int lsTurns = (int) getMotorDegrees() / 20;
    // double rem = getMotorDegrees() / 40 - (double) lsTurns;

    // // get angle from looking up pin distance table and interpolating for
    // // remainder
    // double pinDistance = pinDistances[lsTurns] + (pinDistances[lsTurns + 1] -
    // pinDistances[lsTurns]) * rem;
    // // SmartDashboard.putNumber("PInDist", pinDistance);
    // double valRads = 2 * Math.asin(pinDistance / (2 * pivotDistance));
    // return cameraBaseAngle + Math.toDegrees(valRads);
    // }

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
        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mPidController.setSmartMotionMaxVelocity(maxAcc, slotNumber);
                lastmaxVel = maxVel;
            }

            if (lastallowedErr != allE) {
                mPidController.setSmartMotionAllowedClosedLoopError(allE, slotNumber);
                lastallowedErr = allE;
            }
        }
    }

    // public void calibratePIDV(final double p, final double i, final double d,
    // final double kIz, double allE,
    // int slotNumber) {
    // if (p != lastkPv) {
    // mPidController.setP(p, slotNumber);
    // lastkPv = mPidController.getP(slotNumber);

    // }
    // if (i != lastkIv) {
    // mPidController.setI(i, slotNumber);
    // lastkIv = i;
    // }

    // if (d != lastkDv) {
    // mPidController.setD(d, slotNumber);
    // lastkDv = d;
    // }

    // if (kIz != lastkIzv) {
    // mPidController.setIZone(kIz, slotNumber);
    // lastkIzv = kIz;
    // }

    // if (kMinOutputv != lastkMinOutputv || kMaxOutputv != lastkMaxOutputv) {
    // mPidController.setOutputRange(kMinOutputv, kMaxOutputv, slotNumber);
    // lastkMinOutputv = kMinOutputv;
    // lastkMaxOutputv = kMaxOutputv;
    // }

    // }

    private void setFF_MaxOuts() {
        kFF = .0004;// 10000 rpm = 10000 * .25 deg per rev= 2500 1/2500 = .0004
 //       kFFv = 0.00;
        kMinOutput = -.5;
        kMaxOutput = .6;

        mPidController.setFF(kFF, SMART_MOTION_SLOT);
        // mPidController.setFF(kFFv, VELOCITY_SLOT);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        // mPidController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);
    }

    private void tuneMMGains() {

        double p = Pref.getPref("tIKp");
        double i = Pref.getPref("tIKi");
        double d = Pref.getPref("tIKd");
        double iz = Pref.getPref("tIKiz");
        maxVel = Pref.getPref("tIMaxV");
        maxAcc = Pref.getPref("tIMaxA");
        allowedErr = .01;
        calibratePID(p, i, d, iz, allowedErr, SMART_MOTION_SLOT);

    }

    // private void tuneVelGains() {
    // double p = Pref.getPref("tIKpv");
    // double i = Pref.getPref("tIKiv");
    // double d = Pref.getPref("tIKdv");
    // double iz = Pref.getPref("tIKizv");
    // maxVel = Pref.getPref("tIMaxVv");
    // maxAcc = Pref.getPref("tIMaxAv");

    // allowedErrv = .01;
    // calibratePIDV(p, i, d, iz, allowedErrv, VELOCITY_SLOT);
    // }

    private void checkTune() {

        tuneOn = Pref.getPref("tITune") == 1. && tiltMotorConnected;

        if (tuneOn && !lastTuneOn) {

            tuneMMGains();
            getMMGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

        // vel controller
        // tuneOnv = Pref.getPref("tITunev") == 1. && tiltMotorConnected;

        // if (tuneOnv && !lastTuneOnv) {

        // tuneVelGains();
        // getVelGains();
        // lastTuneOnv = true;
        // }

        // if (lastTuneOnv)
        // lastTuneOnv = tuneOnv;

        // Lock controller

        lockTuneOn = Pref.getPref("tiLTune") != 0.;

    }

    public void getMMGains() {
        ffset = mPidController.getFF(SMART_MOTION_SLOT);
        pset = mPidController.getP(SMART_MOTION_SLOT);
        iset = mPidController.getI(SMART_MOTION_SLOT);
        dset = mPidController.getD(SMART_MOTION_SLOT);
        izset = mPidController.getIZone(SMART_MOTION_SLOT);
        maxAccset = mPidController.getSmartMotionMaxAccel(SMART_MOTION_SLOT);
        maxVelset = mPidController.getSmartMotionMaxVelocity(SMART_MOTION_SLOT);

    }

    // public void getVelGains() {
    // ffsetv = mPidController.getFF(VELOCITY_SLOT);
    // psetv = mPidController.getP(VELOCITY_SLOT);
    // isetv = mPidController.getI(VELOCITY_SLOT);
    // dsetv = mPidController.getD(VELOCITY_SLOT);
    // izsetv = mPidController.getIZone(VELOCITY_SLOT);

    // }

}
