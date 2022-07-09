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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.SimpleCSVLogger;
import frc.robot.commands.AutoCommands.DoNothing;
import frc.robot.commands.RobotDrive.PlotTrajectory;
import frc.robot.subsystems.RevDrivetrain;

/**
 * Add your docs here.
 */
public class FondyFireTrajectory {

        private RevDrivetrain m_drive;

        // LEFT

    //    public final Pose2d leftAutoStart = new Pose2d(6.34, 4.92, Rotation2d.fromDegrees(-42));

        final Pose2d leftCargoRev = new Pose2d(5.34, 4.92, Rotation2d.fromDegrees(Math.PI / 2 - 42));

        public Pose2d leftAutoStartRev = new Pose2d(5.12, 5.97, Rotation2d.fromDegrees(Math.PI / 2 - 42));

        public Trajectory leftPickupRev;

        // CENTER

        public Pose2d centerAutoStartRev = new Pose2d(6.65, 2.83, Rotation2d.fromDegrees(Math.PI / 2 + 34));

      //  public final Pose2d centerAutoStart = new Pose2d(6.65, 2.83, Rotation2d.fromDegrees(34));

        final Pose2d centerCargo = new Pose2d(5.2, 1.9, Rotation2d.fromDegrees(40));

        final Pose2d centerCargo3Shoot = new Pose2d(5.2, 1.9, Rotation2d.fromDegrees(45));

        final Pose2d centerCargoRev = new Pose2d(5.2, 1.9, Rotation2d.fromDegrees(Math.PI / 2 + 35.));

        final Pose2d centerThirdCargoGet = new Pose2d(2.36, .75, Rotation2d.fromDegrees(45));

        final Pose2d centerThirdCargoGetRev = new Pose2d(2.36, .75, Rotation2d.fromDegrees(Math.PI / 2 + 45));

        public Trajectory centerThirdCargoPickUp;

        public Trajectory centerThirdCargoShoot;

        // RIGHT

        public final Pose2d rightAutoStart = new Pose2d(7.4, 2.09, Rotation2d.fromDegrees(-90));

        final Pose2d rightCargoFirstPickup = new Pose2d(7.4, 1.1, Rotation2d.fromDegrees(Math.PI / 2 - 90));

        public Trajectory rightThirdCargoPickupRev1;

        public Trajectory rightThirdCargoPickupRev2;

        public SimpleCSVLogger trajLogger = new SimpleCSVLogger();
        public boolean trajLogInProgress;
        public boolean logTrajItems = true;
        public boolean endFile;

        public FondyFireTrajectory(RevDrivetrain drive) {
                m_drive = drive;

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 11); // 8

                TrajectoryConfig configReversed = new TrajectoryConfig(DriveConstants.kMaxTrajectoryMetersPerSecond,
                                DriveConstants.kMaxTrajectoryAccelerationMetersPerSquared)
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

                leftPickupRev = TrajectoryGenerator.generateTrajectory(

                                leftAutoStartRev,
                                List.of(),
                                leftCargoRev,
                                configReversed);

                centerThirdCargoPickUp = TrajectoryGenerator.generateTrajectory(

                                centerCargoRev,
                                List.of(),
                                centerThirdCargoGetRev,
                                configReversed);

                centerThirdCargoShoot = TrajectoryGenerator.generateTrajectory(
                                centerThirdCargoGet,
                                List.of(),
                                centerCargo3Shoot,
                                configForward);

                rightThirdCargoPickupRev1 = TrajectoryGenerator.generateTrajectory(
                                rightCargoFirstPickup,
                                List.of(),
                                centerCargoRev,
                                configReversed);

                ShuffleboardLayout trajCommands = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryRun", BuiltInLayouts.kList).withPosition(6, 0)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajCommands.add("CenterThirdShoot",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoShoot),
                                                new ParallelCommandGroup(
                                                                new ConditionalCommand(
                                                                                new LogTrajectoryData(drive, this,
                                                                                                centerThirdCargoShoot,
                                                                                                "C3SH"),
                                                                                new DoNothing(),
                                                                                () -> RobotBase.isReal()),
                                                                getRamsete(centerThirdCargoShoot))
                                                                                .andThen(() -> m_drive.tankDriveVolts(0,
                                                                                                0))
                                                                                .andThen(() -> m_drive.trajectoryRunning = false)));

                trajCommands.add("CenterThirdPickup",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoPickUp),
                                                new ParallelCommandGroup(
                                                                new ConditionalCommand(
                                                                                new LogTrajectoryData(drive, this,
                                                                                                centerThirdCargoPickUp,
                                                                                                "C3PU"),
                                                                                new DoNothing(),
                                                                                () -> RobotBase.isReal()),
                                                                getRamsete(centerThirdCargoPickUp))
                                                                                .andThen(() -> m_drive.tankDriveVolts(0,
                                                                                                0))
                                                                                .andThen(() -> m_drive.trajectoryRunning = false)));

                trajCommands.add("RightThirdPickup",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                rightThirdCargoPickupRev1),

                                                new ParallelCommandGroup(

                                                                new ConditionalCommand(new LogTrajectoryData(drive,
                                                                                this,
                                                                                rightThirdCargoPickupRev1, "R3PU"),
                                                                                new DoNothing(),
                                                                                () -> RobotBase.isReal()),
                                                                getRamsete(rightThirdCargoPickupRev1))
                                                                                .andThen(() -> m_drive.tankDriveVolts(0,
                                                                                                0))
                                                                                .andThen(() -> m_drive.trajectoryRunning = false)));

                ShuffleboardLayout trajSetStart = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectorySetStart", BuiltInLayouts.kList).withPosition(8, 0)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajSetStart.add("CenterThirdShootSetStart",

                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoShoot),
                                                new PlotTrajectory(drive, centerThirdCargoShoot)));

                trajSetStart.add("CenterThirdPickupSetStart",

                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                centerThirdCargoPickUp),
                                                new PlotTrajectory(drive, centerThirdCargoPickUp)));

                trajSetStart.add("RightThirdPickupSetStart",
                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                rightThirdCargoPickupRev1),
                                                new PlotTrajectory(drive, rightThirdCargoPickupRev1)));

                ShuffleboardLayout trajInfo = Shuffleboard.getTab("Trajectories")
                                .getLayout("TrajectoryInfo", BuiltInLayouts.kList).withPosition(8, 2)
                                .withSize(2, 2)
                                .withProperties(Map.of("Label position", "LEFT")); // labels for

                trajInfo.addNumber("ShootTrajSecs", () -> centerThirdCargoShoot.getTotalTimeSeconds());
                trajInfo.addNumber("PickupTrajSecs", () -> centerThirdCargoPickUp.getTotalTimeSeconds());
                trajInfo.addNumber("RightTrajSecs", () -> rightThirdCargoPickupRev1.getTotalTimeSeconds());

                if (RobotBase.isReal()) {
                        ShuffleboardTab rob = Shuffleboard.getTab("Trajectories");

                        rob.add("Field", drive.m_field2d).withPosition(0, 0).withSize(5, 4).withWidget("Field");
                }
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
