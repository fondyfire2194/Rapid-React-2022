package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Pref;

public class RevShooterSubsystem extends SubsystemBase {

    public final CANSparkMax mLeftMotor; // NOPMD
    private CANSparkMax mRightMotor; // NOPMD
    private final RelativeEncoder mEncoder;
    private final RelativeEncoder mRightEncoder;
    private final SparkMaxPIDController mPidController;

    private final CANSparkMax m_topRollerMotor;
    public boolean topRollerMotorConnected;
    public boolean haltTopRollerMotor;
    private SparkMaxPIDController m_topPID;
    private RelativeEncoder m_topEncoder;

    public double requiredRPMLast;
    public double requiredRPM;
    public double shotDistance;
    public double shootTime;
    public double shootTimeRemaining;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    public static double kGearing = 1;
    public static double kInertia = 0.008;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, acc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastAcc;
    public double cameraCalculatedSpeed;
    public double pset, iset, dset, ffset, izset;
    public boolean useCameraSpeed;
    public boolean useSetupEntry = true;

    public boolean startShooter;

    public int teleopSetupIndex = 5;

    public double teleopSetupShooterSpeed;

    public boolean shotInProgress;

    public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);

    private final int VELOCITY_SLOT = 0;

    public double[] tiltAngleFromCamerDistance = new double[] { 0.6, 1.2, 1.8, 2.4, 3, 3.6, 4.2, 4.8, 5.4, 6, 6.6, 7.2,
            7.8, 8.4, 9, 9.6, 10.2, 10.8, 11.4, 12, 12.6, 13.2, 13.8, 14.4, 15 };

    public double[] rpmFromCameraDistance = new double[] { 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
            2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900 };

    public double startDistance;

    public int calculatedCameraDistance;

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
    public boolean endFile;
    public boolean endShootFile;
    public boolean isShooting;

    public double testVertOffset;

    public boolean okToShoot;

    public double adjustedCameraMPS;
    public double driverThrottleValue;
    public boolean useProgramSpeed;
    public boolean useDriverSpeed;
    public double programSpeed;

    public double shooterRPMAdder;
    public double shooterRPMChange;
    public double cameraCalculatedTiltPosition;
    public double maxRPM = 4500;
    public double minRPM = 100;
    public double shootCargosRunning;

    public NetworkTableEntry shooterSpeed;
    public boolean correctCargoColor;
    private double topRequiredRPM;
    public double presetRPM;

    public RevShooterSubsystem() {

        mLeftMotor = new CANSparkMax(CANConstants.LEFT_SHOOTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new CANSparkMax(CANConstants.RIGHT_SHOOTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(3.);

        mRightMotor.restoreFactoryDefaults();
        mRightMotor.follow(mLeftMotor, true);

        m_topRollerMotor = new CANSparkMax(CANConstants.TOP_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_topPID = m_topRollerMotor.getPIDController();
        m_topEncoder = m_topRollerMotor.getEncoder();
        m_topEncoder.setPositionConversionFactor(36);
        m_topEncoder.setVelocityConversionFactor(36);
        m_topRollerMotor.restoreFactoryDefaults();
        m_topRollerMotor.setInverted(true);
        setTopRollerBrakeOn(true);
        calibrateTopPID();

        Arrays.asList(mLeftMotor, mRightMotor, m_topRollerMotor)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake mode for faster stop
        Arrays.asList(mLeftMotor, mRightMotor, m_topRollerMotor)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        shooterSpeed = Shuffleboard.getTab("SetupShooter").add("ShooterSpeed",
                3).withWidget("Number Slider")
                .withPosition(0, 3).withSize(4, 1).withProperties(Map.of("Min", 500, "Max",
                        3500))
                .getEntry();

        tuneGains();

        getGains();

        requiredRPM = 2500;

        shootOne = false;

        programSpeed = minRPM;

        if (RobotBase.isReal()) {

            mEncoder.setPositionConversionFactor(1);
            mEncoder.setVelocityConversionFactor(1 / 60);
        }

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(mLeftMotor, DCMotor.getNEO(2));
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkTune();

        SmartDashboard.putString("LeftFaults", "SS");// faultAsBitString());

    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        SmartDashboard.putNumber("VelSim", getRPM());

    }

    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    public void spinAtRpm(double rpm) {
        requiredRPM = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);

    }

    public void spinAtRPM(double rpm) {
        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);

    }

    public double getDriverRPM() {
        return 20 + driverThrottleValue * 20;
    }

    public void runShooter(double rpm) {

        spinAtRPM(-rpm);
    }

    public void runShooterFromCamera() {
        runTopAtVelocity(cameraCalculatedSpeed / 3);
        spinAtRPM(cameraCalculatedSpeed);
    }

    public void runShooterFromPresetPositionSpeed() {
        runTopAtVelocity(presetRPM);
        spinAtRPM(cameraCalculatedSpeed);
    }

    public void moveManually(double speed) {
        if (RobotBase.isReal())
            mLeftMotor.set(speed);
        else
            mPidController.setReference(speed * 12, ControlType.kVoltage, VELOCITY_SLOT);

    }

    public boolean checkCAN() {
        leftMotorConnected = mLeftMotor.getFirmwareVersion() != 0;
        rightMotorConnected = mRightMotor.getFirmwareVersion() != 0;
        topRollerMotorConnected = m_topRollerMotor.getFirmwareVersion() != -1;

        allConnected = leftMotorConnected && rightMotorConnected && topRollerMotorConnected;
        return allConnected;
    }

    public double getRPM() {
        return mEncoder.getVelocity();
    }

    public boolean atSpeed() {

        return Math.abs(requiredRPM + getRPM()) < (requiredRPM * .05);// getmps is -

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
        if (RobotBase.isReal()) {
            mLeftMotor.stopMotor();
            mRightMotor.stopMotor();
            m_topRollerMotor.stopMotor();
        } else {
            mLeftMotor.setVoltage(0);
            mRightMotor.setVoltage(0);
            m_topRollerMotor.setVoltage(0);
        }
        startShooter = false;
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

        m_topPID.setReference(rpm, ControlType.kVelocity, 0);
    }

    public boolean getTopRollerAtSpeed() {
        return Math.abs(topRequiredRPM + getTopRPM()) < (topRequiredRPM * .05);// getrpm is -
    }

    public double getTopRPM() {
        return m_topEncoder.getVelocity();
    }

    public double getTopRoller() {
        return m_topRollerMotor.get();
    }

    public void stopTopRoller() {
        m_topRollerMotor.stopMotor();
    }

    public double getTopRollerMotorAmps() {
        return m_topRollerMotor.getOutputCurrent();
    }

    public void calibrateTopPID() {
        double p = 0.001;
        double i = 0;
        double d = 0;
        double f = 1 / 760;
        double kIz = 0;
        double acc = 1;

        m_topPID.setP(p, 0);

        m_topPID.setI(i, 0);

        m_topPID.setD(d, 0);

        m_topPID.setFF(f, 0);

        m_topPID.setIZone(kIz, 0);

        m_topPID.setOutputRange(-0.5, 0.5, 0);

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
        kMinOutput = -1;

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
