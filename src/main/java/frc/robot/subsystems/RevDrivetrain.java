package frc.robot.subsystems;

import java.util.Arrays;

//import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Pref;
import frc.robot.SimpleCSVLogger;
import frc.robot.Sim.CANEncoderSim;

public class RevDrivetrain extends SubsystemBase {
    // private static final DrivetrainConstants DRIVETRAIN_CONSTANTS = new
    // NeoDrivetrainConstants();

    private final CANSparkMax mLeadLeft; // NOPMD
    private final CANSparkMax mFollowerLeft; // NOPMD

    private final CANSparkMax mLeadRight; // NOPMD
    private final CANSparkMax mFollowerRight; // NOPMD

    public final RelativeEncoder mRightEncoder;
    public final RelativeEncoder mLeftEncoder;

    private final SparkMaxPIDController mLeftPidController;
    private final SparkMaxPIDController mRightPidController;

    private final AHRS mGyro;

    private final Field2d m_field;
    // Simulation
    private CANEncoderSim m_leftEncodersSim;
    private CANEncoderSim m_rightEncodersSim;
    private DifferentialDrivetrainSim m_dts;
    private SimDouble m_simAngle;

    private final DifferentialDrive mDrive;
    private final DifferentialDriveOdometry mOdometry;

    public double leftTargetPosition;
    public double rightTargetPosition;

    public double pset, iset, dset, ffset, izset;
    public double rpset, riset, rdset, rffset, rizset;

    public double startDistance;

    private int SMART_MOTION_SLOT = 0;

    private int VELOCITY_SLOT = 1;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

    public double lastkP, lastkp, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastMaxVel,
            lastMinVel, lastMaxAcc, lastAllowedErr;
    // start NetworkTables

    public boolean tuneOn = false;
    public boolean lastTuneOn;

    public boolean leftLeadConnected;
    public boolean rightLeadConnected;
    public boolean leftFollowerConnected;
    public boolean rightFollowerConnected;
    public boolean allConnected;

    public boolean lockedForVision;
    public boolean leftBurnOK;
    public boolean rightBurnOK;

    public SimpleCSVLogger driveLogger;

    public boolean driveLogInProgress;

    public boolean logDriveItems;

    public boolean endDriveFile;

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

    private int robotStoppedCtr;

    public boolean robotStoppedForOneSecond;

    public RevDrivetrain() {
        mLeadLeft = new CANSparkMax(CANConstants.DRIVETRAIN_LEFT_MASTER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mFollowerLeft = new CANSparkMax(CANConstants.DRIVETRAIN_LEFT_FOLLOWER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeadRight = new CANSparkMax(CANConstants.DRIVETRAIN_RIGHT_MASTER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mFollowerRight = new CANSparkMax(CANConstants.DRIVETRAIN_RIGHT_FOLLOWER,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mLeadLeft.restoreFactoryDefaults();
        mFollowerLeft.restoreFactoryDefaults();
        mLeadRight.restoreFactoryDefaults();
        mFollowerRight.restoreFactoryDefaults();

        mRightEncoder = mLeadRight.getEncoder();
        mLeftEncoder = mLeadLeft.getEncoder();

        if (RobotBase.isReal()) {

            mLeftEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);
            mRightEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);

            mLeftEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);
            mRightEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);

        }
        mGyro = new AHRS();
        mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        mLeftPidController = mLeadLeft.getPIDController();
        mRightPidController = mLeadRight.getPIDController();

        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        leftTargetPosition = 0;
        rightTargetPosition = 0;

        mLeadLeft.setInverted(false);
        mLeadRight.setInverted(true);

        mFollowerLeft.follow(mLeadLeft, false);
        mFollowerRight.follow(mLeadRight, false);
        mLeadLeft.setOpenLoopRampRate(.5);
        mLeadRight.setOpenLoopRampRate(.5);

        mLeadLeft.setClosedLoopRampRate(1);
        mLeadRight.setClosedLoopRampRate(1);

        mDrive = new DifferentialDrive(mLeadLeft, mLeadRight);

        mDrive.setSafetyEnabled(false);

        driveLogger = new SimpleCSVLogger();

        tuneGains();

        setIdleMode(false);

        // Set current limiting on drive train to prevent brown outs
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        kP = .4;
        kI = .1;
        kD = .5;
        maxAcc = 8;
        maxVel = 3;

        if (RobotBase.isSimulation()) {

            // Except for all the extra Java verbiage this is just
            // map(CANEncoderSim::new, motorIDs)
            // private final CANEncoderSim[] motorIDsToCANEncodersSim(Integer[] motorIDs,
            // boolean invert) {
            // return Stream.of(motorIDs).map(m -> new CANEncoderSim(m,
            // invert)).toArray(CANEncoderSim[]::new);

            // CANSparkMaxWithSim mLeftMotorSim = new CANSparkMaxWithSim(CANConstants.DRIVETRAIN_LEFT_MASTER,
            //         CANSparkMaxLowLevel.MotorType.kBrushless);
            
          
            m_leftEncodersSim = new CANEncoderSim(mLeadLeft.getDeviceId(), false);
            m_rightEncodersSim = new CANEncoderSim(mLeadRight.getDeviceId(), false);

            var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
            m_dts = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k12p75,
                    KitbotWheelSize.kSixInch, null);
        }
    }

    @Override
    public void periodic() {

        mOdometry.update(mGyro.getRotation2d(), getLeftDistance(), getRightDistance());
        m_field.setRobotPose(mOdometry.getPoseMeters());
        checkTune();

        if (!isStopped()) {
            robotStoppedCtr = 0;
        }

        if (isStopped() && robotStoppedCtr <= 50)
            robotStoppedCtr++;

        robotStoppedForOneSecond = isStopped() && robotStoppedCtr >= 50;

    }

    @Override
    public void simulationPeriodic() {
        var voltage = RobotController.getInputVoltage();
        var currHdg = m_dts.getHeading();
        m_dts.setInputs(mLeadLeft.get() * voltage,
                mLeadRight.get() * voltage);
        m_dts.update(Constants.kRobotPeriod);
        var newHdg = m_dts.getHeading();
        var hdgDiff = newHdg.minus(currHdg);
        m_leftEncodersSim.setPosition(m_dts.getLeftPositionMeters());
        m_rightEncodersSim.setVelocity(-m_dts.getRightPositionMeters());
        m_simAngle.set(m_simAngle.get() - hdgDiff.getDegrees());
    }

    /////////////////////////////////////
    // Accessors
    /////////////////////////////////////

    public double getLeftDistance() {
        return mLeftEncoder.getPosition();
    }

    public double getRightDistance() {
        return mRightEncoder.getPosition();
    }

    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public double getLeftRate() {
        return mLeftEncoder.getVelocity();
    }

    public double getRightRate() {
        return mRightEncoder.getVelocity();
    }

    public double getRightOut() {
        return mLeadRight.getAppliedOutput();
    }

    public double getRightAmps() {
        return mLeadRight.getOutputCurrent();
    }

    public double getLeftOut() {
        return mLeadLeft.getAppliedOutput();
    }

    public double getLeftAmps() {
        return mLeadLeft.getOutputCurrent();
    }

    public double getRightFollowerOut() {
        return mFollowerRight.getAppliedOutput();
    }

    public double getLeftFollowerOut() {
        return mFollowerLeft.getAppliedOutput();
    }

    public boolean getLeftFollower() {
        return mFollowerLeft.isFollower();
    }

    public boolean getRightFollower() {
        return mFollowerRight.isFollower();
    }

    public void arcadeDrive(double speed, double rotation) {

        if (Math.abs(speed) < .1 || lockedForVision)
            speed = 0;
        if (Math.abs(rotation) < .1)
            rotation = 0;
        mDrive.arcadeDrive(speed, rotation);
    }

    public void tankDriveVolts(double left, double right) {
        mLeadLeft.setVoltage(left);
        mLeadRight.setVoltage(right);
        mDrive.feed();
    }

    public void smartVelocityControlMetersPerSec(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
        mLeftPidController.setReference(leftVelocityMetersPerSec, ControlType.kVelocity, VELOCITY_SLOT);
        mRightPidController.setReference(rightVelocityMetersPerSec, ControlType.kVelocity, VELOCITY_SLOT);
        mDrive.feed();
    }

    public void driveDistance(double leftPosition, double rightPosition) {
        mLeftPidController.setReference(leftPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mDrive.feed();
    }

    public void resetPID() {
        mLeftPidController.setIAccum(0);
        mRightPidController.setIAccum(0);
    }

    public void positionDistance(double leftPosition, double rightPosition) {

        mLeftPidController.setReference(leftPosition, ControlType.kPosition, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kPosition, SMART_MOTION_SLOT);
        mDrive.feed();
    }

    public void resetEncoders() {
        if (RobotBase.isReal()) {
            mLeftEncoder.setPosition(0);
            mRightEncoder.setPosition(0);
        }

    }

    public double getX() {
        return getTranslation().getX();
    }

    public double getY() {
        return getTranslation().getY();
    }

    ///////////////////////////////
    // Life Cycle
    ///////////////////////////////

    public boolean checkCAN() {
        leftLeadConnected = mLeadLeft.getFirmwareVersion() != 0;
        rightLeadConnected = mLeadRight.getFirmwareVersion() != 0;
        leftFollowerConnected = mFollowerLeft.getFirmwareVersion() != 0;
        rightFollowerConnected = mFollowerRight.getFirmwareVersion() != 0;
        allConnected = leftLeadConnected && leftFollowerConnected && rightLeadConnected && rightFollowerConnected;

        return allConnected;
    }

    public void resetAll() {
        resetGyro();
        resetEncoders();
    }

    public double getHeadingDegrees() {
        return mGyro.getAngle();
    }

    public double getYaw() {
        return mGyro.getYaw();
        // return Math.IEEEremainder(mGyro.getAngle(), 360) * -1;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void resetGyro() {
        if (RobotBase.isReal())
            mGyro.reset();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, mGyro.getRotation2d());
        if (RobotBase.isSimulation())
            m_dts.setPose(pose);
    }

    public boolean isStopped() {
        return Math.abs(mLeftEncoder.getVelocity()) < .2;
    }

    public boolean gyroStopped() {
        return !mGyro.isMoving();
    }

    public boolean gyroRotating() {
        return mGyro.isRotating();
    }

    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetPose(Pose2d pose) {
        // The left and right encoders MUST be reset when odometry is reset
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeadingDegrees()));
    }

    public void clearFaults() {
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((CANSparkMax spark) -> spark.clearFaults());

    }

    public int getFaults() {
        return mLeadLeft.getFaults() + mLeadRight.getFaults() + mFollowerLeft.getFaults() + mFollowerRight.getFaults();
    }

    public boolean getInPositionLeft() {
        return Math.abs(leftTargetPosition - getLeftDistance()) < .25;
    }

    public boolean getInPositionRight() {
        return Math.abs(rightTargetPosition - getRightDistance()) < .25;
    }

    public boolean getInPosition() {
        return getInPositionLeft() && getInPositionRight();
    }

    public boolean getStopped() {
        return Math.abs(getLeftRate()) < .5 && Math.abs(getRightRate()) < .5;

    }

    /**
     * Attempts to follow the given drive states using offboard PID.
     *
     * @param left  The left wheel state.
     * @param right The right wheel state.
     */
    public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {

        mLeftPidController.setReference(left.position, ControlType.kPosition);

        m_feedforward.calculate(left.velocity);

        mRightPidController.setReference(right.position, ControlType.kPosition);

        m_feedforward.calculate(right.velocity);
    }

    public void setIdleMode(boolean brake) {
        if (brake) {

            // Set motors to brake when idle. We don't want the drive train to coast.
            Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                    .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
        }

        else {
            // Set motors to brake when idle. We don't want the drive train to coast.
            Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                    .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));

        }
    }

    public void setMaxVel(double maxVel) {

        mLeftPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
        mRightPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    private void setVGains() {
        mLeftPidController.setFF(kFF, VELOCITY_SLOT);
        mRightPidController.setFF(kFF, VELOCITY_SLOT);

        mLeftPidController.setP(kP, VELOCITY_SLOT);
        mRightPidController.setP(kP, VELOCITY_SLOT);

        mLeftPidController.setI(kI, VELOCITY_SLOT);
        mRightPidController.setI(kI, VELOCITY_SLOT);

        mLeftPidController.setD(kD, VELOCITY_SLOT);
        mRightPidController.setD(kD, VELOCITY_SLOT);

    }

    public void close() {
        mLeadLeft.close();
        mFollowerLeft.close();
        mLeadRight.close();
        mFollowerRight.close();
    }

    private void tuneGains() {

        kP = Pref.getPref("dRKp");
        kI = Pref.getPref("dRKi");
        kD = Pref.getPref("dRKd");

        kIz = Pref.getPref("dRKiz");
        kFF = Pref.getPref("dRKff");// 90 rps * .0467 = 4.2 meters per second. 1/4.2 = .238 kff

        setVGains();
        getGains();

    }

    private void getGains() {
        ffset = mLeftPidController.getFF(VELOCITY_SLOT);
        pset = mLeftPidController.getP(VELOCITY_SLOT);
        rffset = mRightPidController.getFF(VELOCITY_SLOT);
        rpset = mRightPidController.getP(VELOCITY_SLOT);

    }

    private void checkTune() {

        tuneOn = Pref.getPref("dRTune") == 1. && allConnected;

        if (tuneOn && !lastTuneOn) {

            tuneGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;
    }

    public void stop() {
        arcadeDrive(0, 0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
    }
}
