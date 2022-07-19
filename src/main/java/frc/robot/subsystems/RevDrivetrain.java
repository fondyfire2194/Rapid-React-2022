package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Pref;
import frc.robot.Sim.CANEncoderSim;

public class RevDrivetrain extends SubsystemBase {
    // private static final DrivetrainConstants DRIVETRAIN_CONSTANTS = new
    // NeoDrivetrainConstants();

    private static final int VELOCITY_SLOT = 0;
    public final CANSparkMax mLeadLeft; // NOPMD
    private final CANSparkMax mFollowerLeft; // NOPMD

    public final CANSparkMax mLeadRight; // NOPMD
    private final CANSparkMax mFollowerRight; // NOPMD

    public final RelativeEncoder mRightEncoder;
    public final RelativeEncoder mLeftEncoder;

    private final AHRS mGyro;

    public final Field2d m_field2d;
    // Simulation
    private CANEncoderSim m_leftEncodersSim;
    private CANEncoderSim m_rightEncodersSim;
    private DifferentialDrivetrainSim m_dts;
    private SimDouble m_simAngle;

    private final DifferentialDrive mDrive;
    public final DifferentialDriveOdometry mOdometry;
    private Pose2d savedPose;
    public double leftTargetPosition;
    public double rightTargetPosition;

    public double pset, iset, dset, ffset, izset;
    public double rpset, riset, rdset, rffset, rizset;

    public double startDistance;

    // private Debouncer trajRunningDelOff = new Debouncer(.1,
    // DebounceType.kFalling);

    // private int SMART_MOTION_SLOT = 0;

    // private int VELOCITY_SLOT = 1;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

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

    public PIDController mleftPID = new PIDController(kP, kI, kD);
    public PIDController mrightPID = new PIDController(kP, kI, kD);

    public SparkMaxPIDController mLeftVel;
    public SparkMaxPIDController mRightVel;

    public double kTurnP = .01;
    public double kTurnI = 0.;
    public double kTurnD = 0.00001;
    public double kTurnIz = 0;

    public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

    private int robotStoppedCtr;

    public boolean robotStoppedForOneSecond;

    public final double pickUpRate = .25;
    public final double positionRate = .4;
    public double rotateStartTime;

    double trajKp = 0;

    public PIDController leftController = new PIDController(trajKp, 0, 0);
    public PIDController rightController = new PIDController(trajKp, 0, 0);

    
    public double leftVolts;
    public double rightVolts;
    public boolean trajectoryRunning;

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

        mLeftVel = mLeadLeft.getPIDController();
        mRightVel = mLeadRight.getPIDController();

        mLeftVel.setFF(.0);
        mLeftVel.setP(.025, 0);
        mRightVel.setFF(.0);
        mRightVel.setP(.025, 0);

        mLeftVel.setOutputRange(-.25, .25, VELOCITY_SLOT);
        mRightVel.setOutputRange(-.25, .25, VELOCITY_SLOT);

        mRightEncoder = mLeadRight.getEncoder();
        mLeftEncoder = mLeadLeft.getEncoder();

        // if (RobotBase.isReal()) {

        mLeftEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);
        mRightEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);

        mLeftEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);
        mRightEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);

     
        mLeadLeft.setOpenLoopRampRate(.01);

        mLeadRight.setOpenLoopRampRate(.01);


        mLeadLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        mLeadLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        mLeadRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        mLeadRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

        mGyro = new AHRS(Port.kUSB);

        mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        m_field2d = new Field2d();

        SmartDashboard.putData("Field", m_field2d);

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

        tuneGains();

        setIdleMode(false);

        // Set current limiting on drive train to prevent brown outs
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        if (RobotBase.isSimulation()) {

            m_leftEncodersSim = new CANEncoderSim(mLeadLeft.getDeviceId(), false);

            m_rightEncodersSim = new CANEncoderSim(mLeadRight.getDeviceId(), false);

            var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

            m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
            m_dts = new DifferentialDrivetrainSim(
                    DriveConstants.kDrivetrainPlant, DCMotor.getNEO(2), 8, .69, 0.0508, null);

            mleftPID.setP(3.);
            mrightPID.setP(3.);

            mleftPID.setD(0);
            mrightPID.setD(0);

        }

    }

    @Override
    public void periodic() {

        mOdometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());

        m_field2d.setRobotPose(mOdometry.getPoseMeters());

        SmartDashboard.putString("Pose", mOdometry.getPoseMeters().toString());

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

        SmartDashboard.putNumber("LPOS", getLeftDistance());
        var voltage = RobotController.getInputVoltage();
        SmartDashboard.putNumber("LOUT", mLeadLeft.get() * voltage);
        SmartDashboard.putNumber("ROUT", mLeadRight.get() * voltage);

        var currHdg = m_dts.getHeading();
        m_dts.setInputs(mLeadLeft.get() * voltage,
                mLeadRight.get() * voltage);
        m_dts.update(Constants.kRobotPeriod);
        var newHdg = m_dts.getHeading();
        var hdgDiff = newHdg.minus(currHdg);

        m_leftEncodersSim.setPosition(m_dts.getLeftPositionMeters());

        m_rightEncodersSim.setPosition(m_dts.getRightPositionMeters());

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
        SmartDashboard.putNumber("ARCrot", rotation);
        if (Math.abs(speed) < .1 || lockedForVision)
            speed = 0;
        if (Math.abs(rotation) < .1)
            rotation = 0;

        mDrive.arcadeDrive(speed, rotation);
    }

    public void rotate(double rotation) {

        if (RobotBase.isReal()) {

            mLeftVel.setReference(rotation * DriveConstants.kMaxTurnRateDegPerS, ControlType.kVelocity, VELOCITY_SLOT);

            mRightVel.setReference(-rotation * DriveConstants.kMaxTurnRateDegPerS, ControlType.kVelocity,
                    VELOCITY_SLOT);

        } else {
            mLeadLeft.set(rotation);
            mLeadRight.set(-rotation);
        }

    }

    public void curvatureDrive(double speed, double rotation) {

        if (Math.abs(speed) < .1)
            speed = 0;
        if (Math.abs(rotation) < .1)
            rotation = 0;
        mDrive.curvatureDrive(speed, rotation, true);
    }

    public void tankDriveVolts(double left, double right) {
        leftVolts = left;
        rightVolts = right;

        if (RobotBase.isReal()) {
            mLeadLeft.setVoltage(left);
            mLeadRight.setVoltage(right);
        } else {
            mLeadLeft.set(left / 12);
            mLeadRight.set(right / 12);
        }
        mDrive.feed();
    }

    public void tankDrive(double left, double right) {
        mDrive.tankDrive(left, right);
        mDrive.feed();
    }

    public double[] driveDistance(double endPosition) {

        double[] temp = { 0, 0 };

        double ival = kI;

        if (RobotBase.isReal() && Math.abs(endPosition - getLeftDistance()) > kIz) {

            ival = 0;

            mleftPID.setI(ival);

            mrightPID.setI(ival);

        }
        SmartDashboard.putNumber("MLKP", mleftPID.getP());

        temp[0] = mleftPID.calculate(getLeftDistance(), endPosition);

        temp[1] = mrightPID.calculate(getRightDistance(), endPosition);

        return temp;
    }

    public void driveLeftSide(double value) {

        mLeadLeft.set(value);

    }

    public void driveRightSide(double value) {

        mLeadRight.set(value);

    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginnig of
     * a match. This will also reset the saved pose since the old pose could be
     * invalidated.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        resetEncoders();
        savedPose = newPose;
        mOdometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
    }

    public void saveCurrentPose() {
        savedPose = getPose();
    }

    public Pose2d getSavedPose() {
        return savedPose;
    }

    public void rotatePose() {
        resetEncoders();
        Pose2d current = getPose();
        mOdometry.resetPosition(current, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetPID() {
        mleftPID.reset();
        mrightPID.reset();
    }

    public void resetEncoders() {
        if (RobotBase.isReal()) {
            mLeftEncoder.setPosition(0);
            mRightEncoder.setPosition(0);
        } else {
            m_leftEncodersSim.setPosition(0);
            m_rightEncodersSim.setPosition(0);
            m_dts.setPose(new Pose2d());

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
        // return mGyro.getYaw();
        return Math.IEEEremainder(mGyro.getAngle(), 360) * -1;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public double getRotationDegrees() {
        return getPose().getRotation().getDegrees();
    }

    public void resetGyro() {
        if (RobotBase.isReal())

            mGyro.reset();

        else

            m_simAngle.set(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        resetGyro();
        mOdometry.resetPosition(pose, mGyro.getRotation2d());
        if (RobotBase.isSimulation())
            m_dts.setPose(pose);
    }

    public boolean isStopped() {
        return Math.abs(mLeftEncoder.getVelocity()) < .2 && Math.abs(mRightEncoder.getVelocity()) < .2;
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

    public void plotTrajectory(Trajectory trajectory) {
        m_field2d.getObject("trajectory").setPoses(
                trajectory.getStates().stream()
                        .map(state -> state.poseMeters)
                        .collect(Collectors.toList()));
    }
    
        public void clearPlot(int l) {

        // Seems to be the only way to clear the lists
        m_field2d.getObject("trajectory").setPoses(Collections.<Pose2d>emptyList());
        for (int i = 1; i <= l; i++)
            m_field2d.getObject("trajectory" + i).setPoses(Collections.<Pose2d>emptyList());
        m_field2d.getObject("waypoints").setPoses(Collections.<Pose2d>emptyList());
        for (int i = 1; i <= l; i++)
            m_field2d.getObject("waypoints" + i).setPoses(Collections.<Pose2d>emptyList());
    }

    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    public void close() {
        mLeadLeft.close();
        mFollowerLeft.close();
        mLeadRight.close();
        mFollowerRight.close();
    }

    private void tuneGains() {

        if (RobotBase.isReal()) {

            kP = Pref.getPref("dRKp");
            kI = Pref.getPref("dRKi");
            kD = Pref.getPref("dRKd");

            kIz = Pref.getPref("dRKiz");

            kTurnP = Pref.getPref("dRTurnkP");

            kTurnI = Pref.getPref("dRTurnkI");

            kTurnD = Pref.getPref("dRTurnkD");

            kTurnIz = Pref.getPref("dRTurnkIz");

            trajKp = Pref.getPref("trajkp");

            mleftPID.setP(kP);
            mleftPID.setI(kI);
            mleftPID.setD(kD);

            mrightPID.setP(kP);
            mrightPID.setI(kI);
            mrightPID.setD(kD);

            leftController.setP(trajKp);
            rightController.setP(trajKp);

        }

    }

    // private void getGains() {

    // pset = mleftPID.getP();
    // dset = mleftPID.getD();
    // iset = mleftPID.getI();

    // }

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

        mLeadLeft.setVoltage(0);
        mLeadRight.setVoltage(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
    }

}
