package frc.robot.subsystems;

import java.util.Arrays;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.FFCSVLogger;
import frc.robot.Pref;
import frc.robot.SimpleCSVLogger;

public class RevShooterSubsystem extends SubsystemBase {

    public final CANSparkMax mLeftMotor; // NOPMD
    private CANSparkMax mRightMotor; // NOPMD
    private final RelativeEncoder mEncoder;
    private final RelativeEncoder mRightEncoder;
    private final SparkMaxPIDController mPidController;

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
    public boolean useSetupSlider;
    public boolean useDriverSpeed;
    public NetworkTableEntry shooterSpeed;
    public FFCSVLogger logger = new FFCSVLogger();
    public FFCSVLogger hubLogger = new FFCSVLogger();

    public boolean startShooter;

    public int teleopSetupIndex = 5;
    public String[] teleopSetupPosition = new String[] { "InitLineStraightOn", "ShieldGenerator",
            "TrenchFrontOfControlPanel", "Trench Behind Control Panel", "Low Goal ", "Not Chosen ", " " };

    public double teleopSetupShooterSpeed;

    public SimpleCSVLogger shootLogger;

    public SimpleCSVLogger shootSetupLogger;

    public boolean logSetup;

    public boolean shotInProgress;

    public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);

    private final int VELOCITY_SLOT = 0;

    public double[] tiltAngleFromCamerDistance = new double[] { 25, 30, 35, 40, 45, 25, 30, 35, 40, 45 };

    public double[] rpmFromCameraDistance = new double[] { 25, 30, 35, 40, 45, 25, 30, 35, 40, 45 };

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
    public boolean endFile;
    public boolean endShootFile;
    public boolean isShooting;

    public boolean shootLogInProgress;
    public double testVertOffset;
    public int itemsLogged;
    public boolean logShooterItems;

    public boolean logSetupFileOpen;
    public boolean okToShoot;

    public double adjustedCameraMPS;
    public double driverThrottleValue;
    public boolean useProgramSpeed;
    public double programSpeed;
    public String[] speedSource = { "Program", "Camera", "Driver", "Setup" };
    public String activeSpeedSource = "Program";
    public double shooterRPMAdder;
    public double shooterRPMChange;
    public double cameraCalculatedTiltOffset;
    public double maxRPM = 4500;
    public double minRPM = 1000;
    public double shootCargosRunning;
    public boolean hubLogInProgress;
    public boolean log;
    public boolean endHubFile;
    public SimpleCSVLogger distCSVLogger = new SimpleCSVLogger();

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

        Arrays.asList(mLeftMotor, mRightMotor).forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake mode for faster stop
        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

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

        if (useCameraSpeed)
            requiredRPM = cameraCalculatedSpeed;
        if (useDriverSpeed)
            requiredRPM = getDriverRPM();
        if (Pref.getPref("IsMatch") == 0) {
            if (useSetupSlider)
                requiredRPM = shooterSpeed.getDouble(20);
        }
        if (useProgramSpeed) {
            requiredRPM = programSpeed;
        }

        if (cameraCalculatedSpeed < 12 || cameraCalculatedSpeed > 40) {
            useCameraSpeed = false;
            useProgramSpeed = true;
        }

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

    public void spinAtMetersPerSec(double metersPerSec) {
        mPidController.setReference(metersPerSec, ControlType.kVelocity, VELOCITY_SLOT);

    }

    public double getDriverRPM() {
        return 20 + driverThrottleValue * 20;
    }

    public void runShooter() {
        requiredRPM += shooterRPMAdder;
        spinAtMetersPerSec(-requiredRPM);
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
        allConnected = leftMotorConnected && rightMotorConnected;
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
        } else {
            mLeftMotor.setVoltage(0);
            mRightMotor.setVoltage(0);
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

    // public double calculateMPSFromDistance(double distance) {

    // double temp = 0;
    // /**
    // * The arrays have distances at which speed step changes
    // *
    // *
    // */
    // int distanceLength = speedBreakMeters.length;
    // double minimumDistance = speedBreakMeters[0];
    // double maximumDistance = speedBreakMeters[distanceLength - 1];

    // double pu;
    // double speedRange;
    // double unitAdder;
    // double distanceRange;

    // if (distance < minimumDistance)
    // distance = minimumDistance;
    // if (distance > maximumDistance)
    // distance = maximumDistance;

    // }
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
