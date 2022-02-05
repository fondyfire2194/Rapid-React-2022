package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.SimpleCSVLogger;

public class RevTurretSubsystem extends SubsystemBase {

    private static final double DEG_PER_MOTOR_REV = HoodedShooterConstants.TURRET_DEG_PER_MOTOR_REV;
    private static final int SMART_MOTION_SLOT = 1;
    public final int POSITION_SLOT = 2;
    public final int VELOCITY_SLOT = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    public double kPv, kIv, kDv, kIzv, kFFv, maxRPMv, maxVelv, minVelv, maxAccv, allowedErrv;
    public double lastkPv, lastkIv, lastkDv, lastkIzv, lastkFFv, lastkMaxOutputv, lastmaxRPMv, lastmaxVelv, lastmv,
            lastmaxAccv, lastallowedErrv;

    private final CANSparkMax m_motor; // NOPMD
    private final RelativeEncoder mEncoder;
    public final SparkMaxPIDController mPidController;
    public final PIDController m_turretLockController = new PIDController(.03, 0, 0);

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
    public boolean endTurretFile;
    public SimpleCSVLogger turretLogger;
    public boolean logTurretItems;
    public boolean turretLogInProgress;
    public double turretVisionTolerance = 1;
    public double testLockFromThrottle;
    public boolean testLock;

    public boolean turretUseVision;
    public double turretOffsetAdder;
    public double turretOffsetChange;

    public RevTurretSubsystem() {
        m_motor = new CANSparkMax(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(5);

        mEncoder.setPosition(0);
        aimCenter();
        setFF_MaxOuts();
        tuneMMGains();
        tuneVelGains();
        getMMGains();
        getVelGains();
        setTurretLockGains();
        getLockGains();
        m_turretLockController.setTolerance(.5);
        setSoftwareLimits();

        if (RobotBase.isReal()) {
            mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);
            mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);

        }

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNeo550(1));
        }
        // if (Pref.getPref("IsMatch") == 0.) {
        // setupHorOffset = Shuffleboard.getTab("SetupTurret").add("SetupHorOffset",
        // 0).withWidget("Number Slider")
        // .withPosition(6, 3).withSize(2, 1).withProperties(Map.of("Min", -5, "Max",
        // 5)).getEntry();
        // }
        turretLogger = new SimpleCSVLogger();
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
        SmartDashboard.putNumber("TUMGETT", m_motor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        SmartDashboard.putNumber("POS", getAngle());

    }

    public boolean checkCAN() {
        return turretMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void moveManually(double speed) {
        targetAngle = getAngle();
        if (RobotBase.isReal()) {
            m_motor.set(speed);
        } else {
            mPidController.setReference(speed * 12, ControlType.kVoltage, POSITION_SLOT);

        }

    }

    public void goToPosition(double angle) {
        mPidController.setReference(angle, ControlType.kPosition, POSITION_SLOT);

    }

    public void positionTurret(double angle, int slotNumber) {
        mPidController.setReference(angle, ControlType.kPosition, slotNumber);

    }

    public double getTargetHorOffset() {
        return targetHorizontalOffset;
    }

    public void goToPositionMotionMagic(double angle) {
        if (RobotBase.isSimulation()) {
            m_motor.set(.1);
        } else
            mPidController.setReference(angle, ControlType.kSmartMotion, SMART_MOTION_SLOT);

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

    public double getOut() {
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
                (float) HoodedShooterConstants.TURRET_MAX_ANGLE);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                (float) HoodedShooterConstants.TURRET_MIN_ANGLE);
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
        return mEncoder.getPosition();
    }

    public void stop() {
        m_motor.set(0);
    }

    public void runAtVelocity(double speed) {
        SmartDashboard.putNumber("PIDATRUN", speed);
        targetAngle = getAngle();
        mPidController.setReference(speed, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public double getIaccum() {
        return mPidController.getIAccum();
    }

    public void calibratePID(final double p, final double i, final double d, final double kIz, int slotNumber) {

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

    public void calibratePIDV(final double p, final double i, final double d, final double kIz, final double allE,
            int slotNumber) {

        if (p != lastkPv) {
            mPidController.setP(p, slotNumber);
            lastkPv = mPidController.getP(slotNumber);

        }
        if (i != lastkIv) {
            mPidController.setI(i, slotNumber);
            lastkIv = mPidController.getI(slotNumber);
        }

        if (d != lastkDv) {
            mPidController.setD(d, slotNumber);
            lastkDv = mPidController.getD(slotNumber);
        }

        if (kIz != lastkIzv) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIzv = mPidController.getIZone(slotNumber);
        }

    }

    private void setFF_MaxOuts() {
        kFF = .0004;
        kFFv = .0;
        kMinOutput = -.5;
        kMaxOutput = .5;
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);
        mPidController.setFF(kFF, SMART_MOTION_SLOT);
        mPidController.setFF(kFFv, VELOCITY_SLOT);

    }

    private void tuneMMGains() {

        double p = Pref.getPref("tURKp");
        double i = Pref.getPref("tURKi");
        double d = Pref.getPref("tURKd");
        double iz = Pref.getPref("tURKiz");
        maxVel = Pref.getPref("tURMaxV");
        maxAcc = Pref.getPref("tURMaxA");
        allowedErr = .1;
        calibratePID(p, i, d, iz, SMART_MOTION_SLOT);

    }

    private void tuneVelGains() {

        double p = Pref.getPref("tURKpv");
        double i = Pref.getPref("tURKiv");
        double d = Pref.getPref("tURKdv");
        double iz = Pref.getPref("tURKizv");

        allowedErrv = .1;
        calibratePIDV(p, i, d, iz, allowedErrv, VELOCITY_SLOT);
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
            tuneMMGains();
            getMMGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

        // vel controller
        tuneOnv = Pref.getPref("tURTunev") == 1. && turretMotorConnected;

        if (tuneOnv && !lastTuneOnv) {

            tuneVelGains();
            getVelGains();
            lastTuneOnv = true;
        }

        if (lastTuneOnv)
            lastTuneOnv = tuneOnv;

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

    public void getMMGains() {
        ffset = mPidController.getFF(SMART_MOTION_SLOT);
        pset = mPidController.getP(SMART_MOTION_SLOT);
        iset = mPidController.getI(SMART_MOTION_SLOT);
        dset = mPidController.getD(SMART_MOTION_SLOT);
        izset = mPidController.getIZone(SMART_MOTION_SLOT);
        maxAccset = mPidController.getSmartMotionMaxAccel(SMART_MOTION_SLOT);
        maxVelset = mPidController.getSmartMotionMaxVelocity(SMART_MOTION_SLOT);

    }

    public void getVelGains() {
        ffsetv = mPidController.getFF(VELOCITY_SLOT);
        psetv = mPidController.getP(VELOCITY_SLOT);
        isetv = mPidController.getI(VELOCITY_SLOT);
        dsetv = mPidController.getD(VELOCITY_SLOT);
        izsetv = mPidController.getIZone(VELOCITY_SLOT);

    }

    public void getLockGains() {
        lpset = m_turretLockController.getP();
        liset = m_turretLockController.getI();
        ldset = m_turretLockController.getD();

    }

}

class turretSim {
    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    // private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
    private final static DCMotor m_armGearbox = DCMotor.getNeo550(1);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double m_armReduction = 600;
    private static final double m_armMass = 5.0; // Kilograms
    private static final double m_armLength = Units.inchesToMeters(30);
    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).
    final static SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            m_armGearbox,
            m_armReduction,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            Units.degreesToRadians(-75),
            Units.degreesToRadians(75),
            m_armMass,
            true,
            VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );

}
