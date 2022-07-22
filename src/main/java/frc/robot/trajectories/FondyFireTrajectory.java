/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

/**
 * Add your docs here.
 */
public class FondyFireTrajectory {

        private RevDrivetrain m_drive;

        NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
        NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

        NetworkTableEntry leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("left_reference");
        NetworkTableEntry sampleTime = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("sampleTime");

        NetworkTableEntry leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("left_measurement");
        NetworkTableEntry leftError = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("left_error");

        NetworkTableEntry rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("right_reference");
        NetworkTableEntry rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("right_measurement");
        NetworkTableEntry rightError = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("right_error");

        NetworkTableEntry angleMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("angle_measurement");

        NetworkTableEntry leftV = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("left_volts");

        NetworkTableEntry rightV = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("right_volts");

        NetworkTableEntry xPosn = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("x_posn");

        NetworkTableEntry yPosn = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("y_posn");

        NetworkTableEntry trajVel = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_vel");
        NetworkTableEntry trajAcc = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_acc");
        NetworkTableEntry trajCurv = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_curv");

        NetworkTableEntry trajX = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_x");
        NetworkTableEntry trajY = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_y");
        NetworkTableEntry trajRot = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_rot");

        NetworkTableEntry trajName = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("traj_name");

        // LEFT
        public Pose2d zeroPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        public Pose2d leftCargoRev = new Pose2d(4.85, 6.27, Rotation2d.fromDegrees(-35));
        public Pose2d leftCargo = new Pose2d(4.85, 6.27, Rotation2d.fromDegrees(145));

        public Pose2d leftOppCargoRev = new Pose2d(6.01, 7.24, Rotation2d.fromDegrees(-135));

        public Pose2d leftAutoStartRev = new Pose2d(6.13, 5.23, Rotation2d.fromDegrees(-41.5));

        public Trajectory leftPickupRev;

        public Trajectory leftHideRev;

        // CENTER

        public Pose2d centerAutoStartRev = new Pose2d(6.56, 2.71, Rotation2d.fromDegrees(+28.4));

        public Pose2d centerHideOppCargo = new Pose2d(4.39, 3.38, Rotation2d.fromDegrees(+132));

        final Pose2d centerCargo3Shoot = new Pose2d(5.2, 1.9, Rotation2d.fromDegrees(45));

        public Pose2d centerCargoRev = new Pose2d(5.13, 1.94, Rotation2d.fromDegrees(37.));

        public Pose2d centerCargo = new Pose2d(5.13, 1.94, Rotation2d.fromDegrees(37.));

        public Pose2d centerThirdCargoGet = new Pose2d(2.36, .75, Rotation2d.fromDegrees(45));

        public Pose2d centerThirdCargoGetRev = new Pose2d(2.36, .75, Rotation2d.fromDegrees(+45));

        public Trajectory centerFirstPickUpRev;

        public Trajectory centerHide;

        public Trajectory centerThirdCargoPickUp;

        public Trajectory centerThirdCargoShoot;

        // RIGHT

        public final Pose2d rightCargoAutoStart = new Pose2d(7.63, 1.9, Rotation2d.fromDegrees(-90));

        final Pose2d rightCargoFirstPickup = new Pose2d(7.63, .75, Rotation2d.fromDegrees(-90));

        final Pose2d leftCenterOppHideAutoStart = new Pose2d(6.08, 4.30, Rotation2d.fromDegrees(32.6));

        final Pose2d leftCenterOppHideCargoRev = new Pose2d(4.53, 3.28, Rotation2d.fromDegrees(24));

        public Trajectory rightThirdCargoPickupRev1;

        public Trajectory rightFirstCargoPickup;

        public Trajectory leftCenterOppHideRev;

        public boolean logTrajItems = true;

        public double time;

        public DifferentialDriveVoltageConstraint autoVoltageConstraint;

        public String activeTrajectoryName;

        public FondyFireTrajectory(RevDrivetrain drive) {

                m_drive = drive;

                autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 11); // 8

                leftPickupRev = TrajectoryGenerator.generateTrajectory(

                                leftAutoStartRev,
                                List.of(),
                                leftCargoRev,
                                withSpeedAndAcceleration(1, .25)
                                                .setReversed(true));

                leftHideRev = TrajectoryGenerator.generateTrajectory(

                                leftCargoRev,
                                List.of(),
                                leftOppCargoRev,
                                withSpeedAndAcceleration(1, .25)
                                                .setReversed(true));

                centerFirstPickUpRev = TrajectoryGenerator.generateTrajectory(

                                centerAutoStartRev,
                                List.of(),
                                centerCargoRev,
                                withSpeedAndAcceleration(.5, .25)
                                                .setReversed(true));

                centerHide = TrajectoryGenerator.generateTrajectory(

                                centerCargoRev,
                                List.of(),
                                centerHideOppCargo,
                                withSpeedAndAcceleration(.75, .25));

                centerThirdCargoPickUp = TrajectoryGenerator.generateTrajectory(

                                centerCargoRev,
                                List.of(),
                                centerThirdCargoGetRev,
                                withSpeedAndAcceleration(.8, .5).setReversed(true));

                centerThirdCargoShoot = TrajectoryGenerator.generateTrajectory(
                                centerThirdCargoGet,
                                List.of(),
                                centerCargo3Shoot,
                                withSpeedAndAcceleration(.8, .5));

                rightFirstCargoPickup = TrajectoryGenerator.generateTrajectory(
                                rightCargoAutoStart,
                                List.of(),
                                rightCargoFirstPickup,
                                withSpeedAndAcceleration(.75, .25));

                rightThirdCargoPickupRev1 = TrajectoryGenerator.generateTrajectory(
                                rightCargoFirstPickup,
                                List.of(),
                                centerCargoRev,
                                withSpeedAndAcceleration(.75, .25).setReversed(true));

                leftCenterOppHideRev = TrajectoryGenerator.generateTrajectory(

                                leftCenterOppHideAutoStart,
                                List.of(),
                                leftCenterOppHideCargoRev,
                                withSpeedAndAcceleration(.75, .25)
                                                .setReversed(true));

                ShuffleboardLayout trajCommands = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryRun", BuiltInLayouts.kList).withPosition(5, 0)
                                .withSize(2, 3)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajCommands.add("LeftPickup", new RunTrajectory(this, drive, leftPickupRev));

                trajCommands.add("CenterPickup", new RunTrajectory(this, drive, centerFirstPickUpRev));

                trajCommands.add("CenterThirdShoot", new RunTrajectory(this, drive, centerThirdCargoShoot));

                trajCommands.add("CenterThirdPickup", new RunTrajectory(this, drive, centerThirdCargoPickUp));

                trajCommands.add("RightThirdPickup", new RunTrajectory(this, drive, rightThirdCargoPickupRev1));

                trajCommands.add("RightFirstPickup", new RunTrajectory(this, drive, rightFirstCargoPickup));

                trajCommands.add("LeftCenterHideOpp", new RunTrajectory(this, drive, leftCenterOppHideRev));

                ShuffleboardLayout trajInfo = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryInfo", BuiltInLayouts.kList).withPosition(7, 0)
                                .withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajInfo.addNumber("LeftPickUpTrajSecs", () -> leftPickupRev.getTotalTimeSeconds());
                trajInfo.addNumber("Cen1PUTrajSecs", () -> centerFirstPickUpRev.getTotalTimeSeconds());
                trajInfo.addNumber("Right1PUTrajSecs", () -> rightFirstCargoPickup.getTotalTimeSeconds());

                trajInfo.addNumber("CenShootTrajSecs", () -> centerThirdCargoShoot.getTotalTimeSeconds());

                trajInfo.addNumber("Right3PUTrajSecs", () -> rightThirdCargoPickupRev1.getTotalTimeSeconds());

                trajInfo.addNumber("LeftWheelSpeed", () -> drive.getWheelSpeeds().leftMetersPerSecond);

                trajInfo.addNumber("RightWheelSpeed", () -> drive.getWheelSpeeds().rightMetersPerSecond);

                trajInfo.addNumber("LeftVolts", () -> drive.leftVolts);

                trajInfo.addNumber("RightVolts", () -> drive.rightVolts);

                trajInfo.addString("LHC", () -> leftHideRev.toString());

                ShuffleboardTab rob = Shuffleboard.getTab("Trajectories");

                if (RobotBase.isReal())
                        rob.add("Field", drive.m_field2d).withPosition(0, 0).withSize(5,
                                        4).withWidget("Field");

        }

        public TrajectoryConfig withSpeedAndAcceleration(

                        double maxSpeedPercent, double maxAccelerationPercent) {

                return new TrajectoryConfig(

                                DriveConstants.kMaxTrajectoryMetersPerSecond *
                                                clamp(maxSpeedPercent, 0, 1),
                                DriveConstants.kMaxTrajectoryAccelerationMetersPerSquared *
                                                clamp(maxAccelerationPercent, 0, 1.5))
                                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                                .addConstraint(autoVoltageConstraint);
        }

        public double clamp(double value, double low, double high) {
                return Math.max(low, Math.min(value, high));
        }

        public RamseteCommand getRamsete(Trajectory traj) {

                RamseteController m_disabledRamsete = new RamseteController();
                m_disabledRamsete.setEnabled(false);

                return new RamseteCommand(traj, m_drive::getPose,
                                // m_disabledRamsete,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                m_drive.leftController, m_drive.rightController,
                                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                // new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                m_drive::tankDriveVolts, m_drive);

        }

}
