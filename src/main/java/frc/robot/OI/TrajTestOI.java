// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.LogTrajectory;
import frc.robot.trajectories.LogTrajectoryData;
import frc.robot.trajectories.ResetOdometryToStartOfTrajectory;

/** Add your docs here. */
public class TrajTestOI {

        public TrajTestOI(RevDrivetrain drive,

                        FondyFireTrajectory traj) {

                ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                robotCommands.add("Reset Enc", new ResetEncoders(drive));
                robotCommands.add("Reset Gyro", new ResetGyro(drive));

                robotCommands.add("Cmd", drive);
                robotCommands.addNumber("Faults", () -> drive.getFaults());
                robotCommands.addNumber("Gyro Yaw", () -> drive.getYaw());
                robotCommands.addNumber("Heading", () -> drive.getHeading());
                robotCommands.addNumber("Rotation", () -> drive.getRotationDegrees());
                robotCommands.add("LogTrajectoryData", new LogTrajectoryData(drive, traj, traj.leftPickupRev, "LPUD"));
                robotCommands.add("LogTrajectoryPoints", new LogTrajectory(traj, traj.leftPickupRev, "LPU"));

                robotCommands.add("LeftPickuptStart",

                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                traj.leftPickupRev),
                                                traj.getRamsete(traj.leftPickupRev)
                                                                .andThen(() -> drive.tankDriveVolts(0,
                                                                                0))
                                                                .andThen(() -> drive.trajectoryRunning = false)));

                robotCommands.add("CenterPickuptStart",

                                new SequentialCommandGroup(
                                                new ResetOdometryToStartOfTrajectory(drive,
                                                                traj.centerFirstPickUpRev),
                                                traj.getRamsete(traj.centerFirstPickUpRev)
                                                                .andThen(() -> drive.tankDriveVolts(0,
                                                                                0))
                                                                .andThen(() -> drive.trajectoryRunning = false)));

                ShuffleboardLayout leftValues = Shuffleboard.getTab("SetupRobot")
                                .getLayout("LeftValues", BuiltInLayouts.kList).withPosition(2, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                leftValues.addNumber("LeftMeters", () -> drive.getLeftDistance());
                leftValues.addNumber("LeftVelMPS", () -> drive.getLeftRate());
                leftValues.addNumber("LeftOut", () -> drive.getLeftOut());
                leftValues.addNumber("LeftAmps", () -> drive.getLeftAmps());
                leftValues.addNumber("LeftFollAmps", () -> drive.getLeftFollowerAmps());

                ShuffleboardLayout rightValues = Shuffleboard.getTab("SetupRobot")
                                .getLayout("RightValues", BuiltInLayouts.kList).withPosition(4, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                rightValues.addNumber("RightMeters", () -> drive.getRightDistance());
                rightValues.addNumber("RightVelMPS", () -> drive.getRightRate());
                rightValues.addNumber("RightOut", () -> drive.getRightOut());
                rightValues.addNumber("RightAmps", () -> drive.getRightAmps());
                rightValues.addNumber("RightFollAmps", () -> drive.getRightFollowerAmps());

                ShuffleboardLayout testValues = Shuffleboard.getTab("SetupRobot")
                                .getLayout("States", BuiltInLayouts.kGrid).withPosition(6, 0)
                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                testValues.addNumber("TestVolts", () -> drive.currentVolts);
                testValues.addNumber("ksVolts", () -> drive.ksVolts);
                testValues.addNumber("kVVolts", () -> drive.kvVolts);
                testValues.addNumber("TrajkpL", () -> drive.leftController.getP());
                testValues.addNumber("TrajkpR", () -> drive.rightController.getP());
                testValues.addNumber("CurrentMPS", () -> drive.currentMPS);
                testValues.addNumber("LeftMPSPerVolt", () -> drive.leftmpspervolt);
                testValues.addNumber("RightMPSPerVolt", () -> drive.rightmpspervolt);
                testValues.addNumber("LeftMPSPSPerVolt", () -> drive.leftmpspspervolt);
                testValues.addNumber("RightMPSPSPerVolt", () -> drive.rightmpspspervolt);

        }

}
