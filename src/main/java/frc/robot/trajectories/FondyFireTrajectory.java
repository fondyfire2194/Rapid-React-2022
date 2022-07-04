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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        public Trajectory leftOppPickup;

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

                                drive.centerCargoRev,
                                List.of(),
                                drive.centerThirdCargoGetRev,
                                configReversed);

                centerThirdCargoShoot = TrajectoryGenerator.generateTrajectory(
                                drive.centerThirdCargoGet,
                                List.of(),
                                drive.centerCargo,
                                configForward);

 
                ShuffleboardLayout trajCommands = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryTest", BuiltInLayouts.kList).withPosition(8, 0)
                                .withSize(2, 1)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajCommands.add("CenterThirdShoot",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoShoot),

                                                getRamsete(centerThirdCargoShoot)
                                                                .andThen(() -> drive.tankDriveVolts(0, 0))
                                                                .andThen(() -> drive.trajectoryRunning = false)));

                trajCommands.add("CenterThirdPickup",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoPickUp),

                                                getRamsete(centerThirdCargoPickUp)));

                ShuffleboardLayout trajInfo = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryInfo", BuiltInLayouts.kList).withPosition(8, 1)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajInfo.addNumber("ShootTrajSecs", () -> centerThirdCargoShoot.getTotalTimeSeconds());
                trajInfo.addNumber("PickupTrajSecs", () -> centerThirdCargoPickUp.getTotalTimeSeconds());
                trajInfo.addNumber("ShootTrajLength", () -> centerThirdCargoShoot.getStates().size());
                trajInfo.addNumber("PickupTrajLength", () -> centerThirdCargoPickUp.getStates().size());

        }

        public RamseteCommand getRamsete(Trajectory traj) {

                RamseteController m_disabledRamsete = new RamseteController();
                m_disabledRamsete.setEnabled(false);

                return new RamseteCommand(traj, m_drive::getPose, // m_disabledRamsete,
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
