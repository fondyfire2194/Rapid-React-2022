package frc.robot.subsystems;

import java.util.Map;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.Sim.CANEncoderSim;
import frc.robot.Sim.CANSparkMaxWithSim;
import frc.robot.Pref;

public class RevTiltSubsystem extends SubsystemBase {

    public final int VELOCITY_SLOT = 0;
    // public final int SMART_MOTION_SLOT = 1;
    // public final int VELOCITY_SLOT = 2;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    public final CANSparkMaxWithSim m_motor;
    private final RelativeEncoder mEncoder;
    public final SparkMaxPIDController mVelController;
    public SparkMaxLimitSwitch m_reverseLimit;
    public SparkMaxLimitSwitch m_forwardLimit;
    private CANEncoderSim mEncoderSim;
    private PIDController mPosController;

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

    public double psetv, isetv, dsetv, ffsetv, izsetv, maxAccset, maxVelset;
    public double psetp, isetp, dsetp, izsetp;

    public boolean tuneOnv = false;
    public boolean lastTuneOnv;

    private boolean posTuneOn;
    private boolean lastPosTuneOn;

    public boolean tiltMotorConnected;
    private double maxAdjustMeters = .5;
    private double minAdjustMeters = -.5;
    public int faultSeen;
    public boolean visionOnTarget;
    public double driverAdjustAngle;
    public double driverAdjustDistance;
    public double adjustMeters = .16;// 6"

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
    public double tiltOffsetAdder;
    public double tiltOffsetChange;
    public double cameraCalculatedTiltPosition;
    public double presetPosition;
    public NetworkTableEntry tiltTarget;

    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    LinearSystem<N2, N1, N1> m_tiltPosition = LinearSystemId
            .identifyPositionSystem(TiltConstants.kVVoltSecondsPerRotation, TiltConstants.kA);
    LinearSystemSim<N2, N1, N1> m_tiltPositionSim = new LinearSystemSim<>(m_tiltPosition);

    /** 
     * 
     * 
     * 
     * 
     * 
     */

    public RevTiltSubsystem() {
        m_motor = new CANSparkMaxWithSim(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mVelController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(1);

        mPosController = new PIDController(.003, 0, 0);
        aimCenter();
        mEncoder.setPosition(0);
        targetAngle = 0;
        positionResetDone = false;

        mEncoder.setPositionConversionFactor(degreesPerRev);
        mEncoder.setVelocityConversionFactor(degreesPerRev / 60);

        m_motor.setSmartCurrentLimit(20);

        if (RobotBase.isReal()) {
            setFF_MaxOuts();
            setPosGains();
            setVelGains();
        }
        m_motor.setIdleMode(IdleMode.kBrake);

        m_reverseLimit = m_motor.getReverseLimitSwitch(Type.kNormallyClosed);

        m_reverseLimit.enableLimitSwitch(true);

        m_forwardLimit = m_motor.getForwardLimitSwitch(Type.kNormallyClosed);

        m_forwardLimit.enableLimitSwitch(true);

        if (RobotBase.isSimulation()) {

            mEncoderSim = new CANEncoderSim(m_motor.getDeviceId(), false);

        }

        if (RobotBase.isReal() && m_reverseLimit.isPressed()) {

            resetAngle();
        }

        enableSoftLimits(false);

        setSoftwareLimits();

        tiltTarget = Shuffleboard.getTab("SetupTilt")
                .add("TargetDegrees", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 3).withSize(2, 1)
                .withProperties(Map.of("min", 0, "max", 14))
                .getEntry();


                SmartDashboard.putNumber("TIDeg/Rev", degreesPerRev);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (RobotBase.isReal())

            checkTune();

        if (RobotBase.isReal() && DriverStation.isDisabled())
            targetAngle = getAngle();

        if (faultSeen != 0)
            faultSeen = getFaults();

        SmartDashboard.putBoolean("RefDone", positionResetDone);

    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our tilt is doing
        // First, we set our "inputs" (voltages)
        m_tiltPositionSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_tiltPositionSim.update(0.020);
        // Finally, we set our simulated encoder's readings and simulated battery
        // voltage
        mEncoderSim.setPosition(m_tiltPositionSim.getOutput(0));
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_tiltPositionSim.getCurrentDrawAmps()));

    }

    public boolean checkCAN() {
        return tiltMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public double getIaccum() {
        return mVelController.getIAccum();
    }

    public void close() {
        m_motor.close();
    }

    public void moveAtVelocity(double degPerSec) {

        if (RobotBase.isReal())

            mVelController.setReference(degPerSec, ControlType.kVelocity, VELOCITY_SLOT);

        else

            moveManually(degPerSec / maxVel);

    }

    public void moveManually(double speed) {

        m_motor.set(speed);
    }

    public void goToPosition(double degrees) {

        double pidout = 0;

        pidout = mPosController.calculate(getAngle(), degrees);

        SmartDashboard.putNumber("TIPIDOUT", pidout);

        if (RobotBase.isReal())

            moveAtVelocity(pidout * maxVel);

        else

            m_motor.set(pidout);
    }

    public void resetAngle() {

        mEncoder.setPosition(0);

        targetAngle = 0;

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

        if (RobotBase.isReal())

            return getMotorDegrees();
        else
            return getMotorDegrees() * degreesPerRev;
    }

    public double getCameraAngle() {
        return getAngle();
    }

    public double getMotorOut() {
        return m_motor.get();
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

    public boolean onPlusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public void stop() {

        m_motor.set(0);

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
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mVelController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }

    }

    private void setFF_MaxOuts() {

        kFF = 0;// 10,000 rpm = (10000/60) * .03645 = 6.0 1/6.0 = .167
        kMinOutput = -.5;
        kMaxOutput = .6;

        mVelController.setFF(kFF, VELOCITY_SLOT);

        mVelController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);

    }

    private void setVelGains() {
        kFF = Pref.getPref("tiVKff");// 9,000/60 rps= 150rps * .036deg/rev = 5.4 and ff = 1/5.4 = .185
        double p = Pref.getPref("tiVKp");
        double i = Pref.getPref("tiVKi");
        double d = Pref.getPref("tiVKd");
        double iz = Pref.getPref("tiVKiz");
        kMinOutput = -.75;
        kMaxOutput = .75;
        mVelController.setOutputRange(kMinOutput, kMaxOutput, VELOCITY_SLOT);
        maxVel = Pref.getPref("tiVMaxV");// deg per sec
        maxAcc = Pref.getPref("tiVMaxA");// deg per sec per sec
        allowedErr = .1;
        calibratePID(p, i, d, iz, allowedErr, VELOCITY_SLOT);

    }

    private void setPosGains() {

        mPosController.setP(Pref.getPref("TiPkP"));
        mPosController.setI(Pref.getPref("TiPkI"));
        mPosController.setD(Pref.getPref("TiPkD"));
        izsetp = Pref.getPref("TiPkIZ");
        mPosController.setIntegratorRange(-izsetp, izsetp);
        mPosController.setTolerance(.5);
    }

    private void checkTune() {

        tuneOnv = Pref.getPref("tiVTune") == 1. && tiltMotorConnected;

        if (tuneOnv && !lastTuneOnv) {
            setVelGains();
            getVelGains();
            lastTuneOnv = true;
        }

        if (lastTuneOnv)
            lastTuneOnv = tuneOnv;

        // posn controller

        posTuneOn = Pref.getPref("tiPTune") != 0.;

        if (posTuneOn && !lastPosTuneOn) {
            setPosGains();
            lastPosTuneOn = true;
        }

        if (lastPosTuneOn)
            lastPosTuneOn = posTuneOn;

    }

    public void getVelGains() {
        ffsetv = mVelController.getFF(VELOCITY_SLOT);
        psetv = mVelController.getP(VELOCITY_SLOT);
        isetv = mVelController.getI(VELOCITY_SLOT);
        dsetv = mVelController.getD(VELOCITY_SLOT);
        izsetv = mVelController.getIZone(VELOCITY_SLOT);

    }

}
