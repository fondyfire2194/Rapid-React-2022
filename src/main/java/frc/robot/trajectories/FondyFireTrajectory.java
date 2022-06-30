/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Pref;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

/**
 * Add your docs here.
 */
public class FondyFireTrajectory {

        private RevDrivetrain m_drive;
        public Trajectory centerThirdCargoPickUp;
        public Trajectory centerThirdCargoShoot;



        public FondyFireTrajectory(RevDrivetrain drive) {
                m_drive = drive;

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 11); // 8

                TrajectoryConfig configReversed = new TrajectoryConfig(DriveConstants.kMaxTrajectoryMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
                configReversed.setReversed(true);

                TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.kMaxTrajectoryMetersPerSecond,
                                DriveConstants.kMaxTrajectoryAccelerationMetersPerSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                centerThirdCargoPickUp = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(5.11, 2.12, new Rotation2d(Units.degreesToRadians(Math.PI / 2 + 25))),
                                List.of(),
                                new Pose2d(2.71, 1.25, new Rotation2d(Units.degreesToRadians(Math.PI / 2 + 45))),
                                configReversed);

                centerThirdCargoShoot = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(2.85, 1.38, new Rotation2d(Units.degreesToRadians(45))),
                                List.of(),
                                new Pose2d(5.11, 2.12, new Rotation2d(Units.degreesToRadians(25))),

                                configForward);

                SmartDashboard.putNumber("ShootTrajTime", centerThirdCargoShoot.getTotalTimeSeconds());
                SmartDashboard.putNumber("PickupTrajTime", centerThirdCargoPickUp.getTotalTimeSeconds());
                SmartDashboard.putNumber("PUTrjsize", centerThirdCargoPickUp.getStates().size());
                SmartDashboard.putNumber("ShootTrajSize", centerThirdCargoShoot.getStates().size());
        }

        public RamseteCommand getRamsete(Trajectory traj) {
                return new RamseteCommand(traj, m_drive::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                m_drive::tankDriveVolts, m_drive);
        }

}
