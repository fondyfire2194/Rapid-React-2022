// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;

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
                testValues.addNumber("CurrentMPSCmd", () -> drive.currentMPSCommand);
                testValues.addNumber("kv", () -> drive.voltsPerMPS);
                testValues.addNumber("accVolts", () -> drive.accelerationVolts);

                testValues.addNumber("KA calc", () -> drive.kA);

        }

}
