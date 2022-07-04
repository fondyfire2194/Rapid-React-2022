// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.RevDrivetrain;

/** Add your docs here. */
public class TrajectoryDebug implements Sendable {

        private RevDrivetrain m_drive;
        private FondyFireTrajectory m_fftraj;

        public TrajectoryDebug(RevDrivetrain drive, FondyFireTrajectory fftrajFireTrajectory) {
                m_fftraj = fftrajFireTrajectory;
                m_drive = drive;

        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("TrajectorySequence");
                builder.addStringProperty("Name", () -> m_fftraj.centerThirdCargoPickUp.toString(), null);
                builder.addDoubleProperty("Length", () -> m_fftraj.centerThirdCargoPickUp.getTotalTimeSeconds(), null);

            
                builder.addDoubleProperty("LeftSetpoint", () -> m_drive.leftController.getSetpoint(), null);
                builder.addDoubleProperty("LeftWheelSpeed", () -> m_drive.getWheelSpeeds().leftMetersPerSecond, null);
  
                builder.addDoubleProperty("RightSetpoint", () -> m_drive.rightController.getSetpoint(), null);
                builder.addDoubleProperty("RightWheelSpeed", () -> m_drive.getWheelSpeeds().rightMetersPerSecond, null);
  
                builder.addDoubleProperty("GyroHeading", () -> m_drive.getHeading(), null);


        }
}
