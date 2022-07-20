package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Pref;
import frc.robot.Sim.CANEncoderSim;
import frc.robot.Sim.CANSparkMaxWithSim;

public class RevShooterSubsystem extends SubsystemBase {

    public final CANSparkMaxWithSim mLeftMotor; // NOPMD
    private CANSparkMaxWithSim mRightMotor; // NOPMD
    private final RelativeEncoder mEncoder;
    private final RelativeEncoder mRightEncoder;
    private final SparkMaxPIDController mPidController;

    private CANEncoderSim mEncoderSim;

    private final CANSparkMaxWithSim m_topRollerMotor;
    private CANEncoderSim m_topEncoderSim;
    public boolean topRollerMotorConnected;
    public boolean haltTopRollerMotor;
    private SparkMaxPIDController m_topPID;
    private RelativeEncoder m_topEncoder;

    public double requiredRPMLast;
    // public double requiredRPM;
    public double shotDistance;
    public double shootTime;
    public double shootTimeRemaining;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    private FlywheelSim flywheel;
    private FlywheelSim topRollerSim;
    public static double kGearing = 1;
    public static double kInertia = 0.008;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, acc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastAcc;
    public double cameraCalculatedSpeed;
    public double pset, iset, dset, ffset, izset;

    public int fromThrottle = 0;
    public int fromCamera = 1;
    public int fromPreset = 2;

    public boolean startShooter;

    public int teleopSetupIndex = 5;

    public double teleopSetupShooterSpeed;

    public int shootLocation;

    public boolean wrongCargoColor;

    public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);

    public final int VELOCITY_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public final int POSITION_SLOT = 2;

    public double[] rpmFromCameraDistance = {
            1850, 1850, 1850, 2150, 1900,//1 - 5 ft 4 degrees
            1950, 2100, 2250,//6-8 ft 11 degrees
            2350, 2450, 2700, 2900, 2900, 2950, // 9-14 ft 14 degrees
            3100, 3250, 3450, 3450, 4500, 5000 };// 15-20ft 17 degrees

    public double startDistance;

    public double calculatedCameraDistance;

    public boolean tuneOn = false;
    public boolean lastTuneOn;
    public boolean leftMotorConnected;
    public boolean rightMotorConnected;
    public boolean allConnected;
    public double cameraAngleCalculatedSpeed;

    public boolean driverOKShoot;
    public boolean burnOK;
    public double shooterRecoverTime = .5;
    public boolean shootOne;

    public boolean isShooting;

    public boolean okToShoot;

    public double adjustedCameraMPS;
    public double driverThrottleValue;

    public double[] shooterRPMAdder = { 0, 0, 0, 0 };
    public double shooterRPMChange;
    public double cameraCalculatedTiltPosition;
    public double maxRPM = 5500;
    public double minRPM = 100;
    public double shootCargosRunning;

    public boolean correctCargoColor;
    public double topRequiredRPM;
    public double presetRPM = 1750;
    public int shootValuesSource = 0;
    public boolean haltTopRoller;
    public boolean isAtSpeed;
    public String presetLocationName = "Not Assigned";
    public String shootValuesSourceName = "Not_Assigned";
    public int presetSource = 2;
    public int cameraSource = 1;
    public int setUpSource = 0;
    public String shootModeName = "Unassigned";
    public boolean runContinuous;

    public RevShooterSubsystem() {

        mLeftMotor = new CANSparkMaxWithSim(CANConstants.LEFT_SHOOTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new CANSparkMaxWithSim(CANConstants.RIGHT_SHOOTER_MOTOR,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setInverted(true);
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(3.);
        mLeftMotor.enableVoltageCompensation(12);

        mRightMotor.restoreFactoryDefaults();
        mRightMotor.follow(mLeftMotor, true);

        
        
        mLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        mLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

        
        mRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        mRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

        m_topRollerMotor = new CANSparkMaxWithSim(CANConstants.TOP_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_topPID = m_topRollerMotor.getPIDController();
        m_topEncoder = m_topRollerMotor.getEncoder();
        m_topRollerMotor.restoreFactoryDefaults();

        m_topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        m_topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_topRollerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        m_topEncoder.setPositionConversionFactor(.1);
        m_topEncoder.setVelocityConversionFactor(.1);

        m_topRollerMotor.setInverted(false);
        setTopRollerBrakeOn(true);

        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake mode for faster stop
        Arrays.asList(mLeftMotor, mRightMotor, m_topRollerMotor)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        tuneGains();

        getGains();

        calibrateTopPID();

        // requiredRPM = 2500;

        shootOne = false;

        if (RobotBase.isReal()) {

            mEncoder.setPositionConversionFactor(1);

            mEncoder.setVelocityConversionFactor(1);
        }

        if (RobotBase.isSimulation()) {

            flywheel = new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
                    DCMotor.getNEO(2),
                    4096);
            mEncoderSim = new CANEncoderSim(mLeftMotor.getDeviceId(), false);

            topRollerSim = new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(ShooterConstants.kV, ShooterConstants.kA),
                    DCMotor.getNeo550(1),
                    4096);
            m_topEncoderSim = new CANEncoderSim(m_topRollerMotor.getDeviceId(), false);

        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkTune();

    }

    @Override
    public void simulationPeriodic() {
        var vin = mLeftMotor.getAppliedOutput() * RobotController.getInputVoltage();

        flywheel.setInputVoltage(vin);
        flywheel.update(0.02);
        mEncoderSim.setVelocity(flywheel.getAngularVelocityRPM());

        vin = m_topRollerMotor.getAppliedOutput() * RobotController.getInputVoltage();

        topRollerSim.setInputVoltage(vin);
        topRollerSim.update(0.02);
        m_topEncoderSim.setVelocity(topRollerSim.getAngularVelocityRPM());

    }

    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    public void spinAtRPM(double rpm) {
        // requiredRPM = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);

    }

    public void runShooterPlusRoller(double rpm) {

        if (wrongCargoColor)

            rpm = 800;

        spinAtRPM(rpm);

        runTopAtVelocity(Pref.getPref("TopRollShootRPM"));
    }

    public void reverseUpperRoller() {
        runTopAtVelocity(-200);
    }

    public void stopUpperRoller() {
        m_topRollerMotor.stopMotor();
    }

    public void moveManually(double speed) {
        mLeftMotor.set(speed);
    }

    public boolean checkCAN() {
        leftMotorConnected = mLeftMotor.getFirmwareVersion() != 0;
        rightMotorConnected = mRightMotor.getFirmwareVersion() != 0;
        topRollerMotorConnected = m_topRollerMotor.getFirmwareVersion() != -1;

        allConnected = leftMotorConnected && rightMotorConnected && topRollerMotorConnected;
        return allConnected;
    }

    public double getRPM() {
        return Math.round(mEncoder.getVelocity());
    }

    public boolean getShooterAtSpeed() {
        return isAtSpeed;

    }

    public boolean getShooterIsShooting() {
        return isShooting;
    }

    public void jogLeftMotor() {
        mPidController.setReference(.1, ControlType.kDutyCycle);
    }

    public double getLeftAmps() {
        return mLeftMotor.getOutputCurrent();
    }

    public double getRightAmps() {
        return mRightMotor.getOutputCurrent();
    }

    public double getMatchTimeRemaining() {
        return DriverStation.getMatchTime();
    }

    public void stop() {

        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
        mLeftMotor.set(0);

        m_topRollerMotor.stopMotor();
        m_topRollerMotor.set(0);

        startShooter = false;
    }

    public double getRPMFromSpeedSource() {

        switch (shootValuesSource) {
            case 0:
                return getRPMfromThrottle();
            case 1:
                return cameraCalculatedSpeed;
            case 2:
                return presetRPM;
            default:
                return 500;

        }

    }

    public double getRPMfromThrottle() {
        double maxrpm = 5500;
        double minrpm = 500;
        double rpmRange = maxrpm - minrpm;
        return minrpm + rpmRange * driverThrottleValue;
    }

    public double getLeftPctOut() {
        return mLeftMotor.get();
    }

    public void clearLeftFaults() {
        mLeftMotor.clearFaults();

    }

    public void clearRightFaults() {

        mRightMotor.clearFaults();
    }

    public void clearFaults() {
        clearLeftFaults();
        clearRightFaults();
    }

    public int getLeftFaults() {
        return mLeftMotor.getFaults();
    }

    public String faultAsBitString() {
        return Integer.toBinaryString(getLeftFaults());
    }

    public int getRightFaults() {
        return mLeftMotor.getFaults();
    }

    public int getFaults() {
        return mLeftMotor.getFaults() + mRightMotor.getFaults();
    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            double acc, int slotNumber) {

        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = p;
        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = i;
        }
        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = d;
        }

        if (f != lastkFF) {
            mPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIz = kIz;
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (acc != lastAcc) {
            mLeftMotor.setClosedLoopRampRate(acc);
            lastAcc = acc;
        }

    }

    public double getTargetDistance() {
        return calculatedCameraDistance;
    }

    public double getBatteryVoltage() {
        return pdp.getVoltage();
    }

    public double getTotalAmpa() {
        return pdp.getTotalCurrent();
    }

    public double getTotalEnergy() {
        return pdp.getTotalEnergy();
    }

    public double getTotalPower() {
        return pdp.getTotalPower();
    }

    public double getTemperature() {
        return pdp.getTemperature();
    }

    public void setTopRollerBrakeOn(boolean on) {
        if (on) {
            m_topRollerMotor.setIdleMode(IdleMode.kBrake);
        } else {
            m_topRollerMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void runTopAtVelocity(double rpm) {

        topRequiredRPM = rpm;

        m_topPID.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public boolean getTopRollerAtSpeed() {
        return isAtSpeed;
    }

    public double getTopRPM() {
        return Math.round(m_topEncoder.getVelocity() * 10) / 10;
    }

    public double getTopRoller() {
        return m_topRollerMotor.getAppliedOutput();
    }

    public void stopTopRoller() {
        m_topRollerMotor.stopMotor();
    }

    public double getTopRollerMotorAmps() {
        return m_topRollerMotor.getOutputCurrent();
    }

    public void calibrateTopPID() {
        double p = Pref.getPref("Rollers_kP");
        double i = 0;
        double d = Pref.getPref("Rollers_kD");
        double f = .0009;// 1/1000 (10,000 rpm through 10:1 gearing)
        double kIz = 0;
        double acc = .5;

        m_topPID.setP(p, VELOCITY_SLOT);

        m_topPID.setI(i, VELOCITY_SLOT);

        m_topPID.setD(d, VELOCITY_SLOT);

        m_topPID.setFF(f, VELOCITY_SLOT);

        m_topPID.setIZone(kIz, VELOCITY_SLOT);

        m_topPID.setOutputRange(-1., 1., VELOCITY_SLOT);

        m_topRollerMotor.setClosedLoopRampRate(acc);

    }

    private void tuneGains() {
        fixedSettings();
        double f = Pref.getPref("sHff");// 5700 rpm = 95 rps = 95 * .638 =
        double p = Pref.getPref("sHkp");
        double i = Pref.getPref("sHki");
        double d = Pref.getPref("sHkd");
        double iz = Pref.getPref("sHkiz");
        double acc = 1;

        calibratePID(p, i, d, f, iz, acc, VELOCITY_SLOT);

    }

    private void fixedSettings() {

        kMaxOutput = 1;
        kMinOutput = 0;

    }

    private void checkTune() {
        REVLibError burnError = REVLibError.kError;

        if (Pref.getPref("sHTune") == 5. && leftMotorConnected) {
            burnOK = false;
            burnError = mLeftMotor.burnFlash();
            burnOK = burnError == REVLibError.kOk;
            getGains();
        }

        tuneOn = Pref.getPref("sHTune") == 1. && leftMotorConnected;

        if (tuneOn && !lastTuneOn) {
            tuneGains();
            getGains();
            calibrateTopPID();
            lastTuneOn = true;
        }
        if (lastTuneOn)
            lastTuneOn = tuneOn;

    }

    public void getGains() {
        ffset = mPidController.getFF(0);
        pset = mPidController.getP(0);
        iset = mPidController.getI(0);
        dset = mPidController.getD(0);
        izset = mPidController.getIZone();

    }

    public void shootOne() {
        shootOne = false;
    }

    public void shootAll() {
        shootOne = false;
    }

    public void setOKShootDriver() {
        driverOKShoot = true;
    }

    public void setNotOKShootDriver() {
        driverOKShoot = false;
    }

}
