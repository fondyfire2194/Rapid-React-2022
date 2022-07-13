// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.RevDrivetrain;

/** Add your docs here. */
public class TroubleshootTrajectories implements Sendable {

        private RevDrivetrain m_drive;
        private FondyFireTrajectory m_fftraj;

        NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
        NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

        NetworkTableEntry leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
                        .getEntry("left_reference");
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

        public TroubleshootTrajectories(RevDrivetrain drive, FondyFireTrajectory fftraj) {
                m_drive = drive;
                m_fftraj = fftraj;
        }

        public void periodic() {

                var translation = m_drive.mOdometry.getPoseMeters().getTranslation();
                m_xEntry.setNumber(translation.getX());
                m_yEntry.setNumber(translation.getY());

                leftMeasurement.setNumber(m_drive.getWheelSpeeds().leftMetersPerSecond);
                leftReference.setNumber(m_drive.leftController.getSetpoint());
                leftError.setNumber(
                                m_drive.leftController.getSetpoint() - m_drive.getWheelSpeeds().leftMetersPerSecond);

                rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
                rightReference.setNumber(m_drive.rightController.getSetpoint());
                rightError.setNumber(
                                m_drive.rightController.getSetpoint() - m_drive.getWheelSpeeds().rightMetersPerSecond);

                angleMeasurement.setNumber(m_drive.getHeading());

        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Trajectory");
                builder.addStringProperty("Trajectory Name", () -> m_fftraj.activeTrajectoryName, null);
                builder.addDoubleProperty("LeftVolts", () -> m_drive.leftVolts, null);
                builder.addDoubleProperty("RightVolts", () -> m_drive.rightVolts, null);
                builder.addDoubleProperty("LeftMPS", () -> m_drive.getWheelSpeeds().leftMetersPerSecond, null);
                builder.addDoubleProperty("RightMPS", () -> m_drive.getWheelSpeeds().rightMetersPerSecond, null);
                builder.addDoubleProperty("X Posn", () -> m_drive.getX(), null);
                builder.addDoubleProperty("Y Posn", () -> m_drive.getY(), null);
                builder.addDoubleProperty("Rotation", () -> m_drive.getHeading(), null);

                builder.addDoubleProperty("LeftAmps", () -> m_drive.getLeftAmps(), null);
                builder.addDoubleProperty("RightAmps", () -> m_drive.getRightAmps(), null);

        }

}
