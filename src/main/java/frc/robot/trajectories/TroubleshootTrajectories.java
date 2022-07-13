// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.RevDrivetrain;

/** Add your docs here. */
public class TroubleshootTrajectories {

    private RevDrivetrain m_drive;
    private FondyFireTrajectory m_fftraj;

    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

    NetworkTableEntry leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
            .getEntry("left_reference");
    NetworkTableEntry leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
            .getEntry("left_measurement");
    NetworkTableEntry rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting")
            .getEntry("right_reference");
    NetworkTableEntry rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting")
            .getEntry("right_measurement");

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

        rightMeasurement.setNumber(m_drive.getWheelSpeeds().rightMetersPerSecond);
        rightReference.setNumber(m_drive.rightController.getSetpoint());

    }
}
