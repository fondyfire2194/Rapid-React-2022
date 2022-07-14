// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldMap;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;

/** Add your docs here. */
public class SetUpOI {

        public static boolean showTurret = true;
        public static boolean showTilt = true;
        public static boolean showShooter = true;
        public static boolean showRobot = true;
        public static boolean showTransport = true;
        public static boolean showClimber = true;
        public static boolean showSubsystems = true;
        public static boolean showIntake = true;

        public double timeToStart;

        public SetUpOI( RevDrivetrain drive,
                       
                        LimeLight limelight, 
                        FondyFireTrajectory traj) {

               
                if (showRobot) {

                        ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        robotCommands.add("Reset Enc", new ResetEncoders(drive));
                        robotCommands.add("Reset Gyro", new ResetGyro(drive));

                        robotCommands.add("ClearFaults", new ClearRobFaults(drive));
                        robotCommands.add("Stop Robot", new StopRobot(drive));
                        robotCommands.add("To 3", new PositionStraight(drive, +3, .5));
                        robotCommands.add("To -3", new PositionStraight(drive, -3, .5));
                        robotCommands.add("To 2", new PositionStraight(drive, 2, .5));
                        robotCommands.add("To -1.6", new PositionStraight(drive, -1.6, drive.pickUpRate));
                        robotCommands.add("To 0", new PositionStraight(drive, 0, .5));
                        robotCommands.add("Cmd", drive);

                        ShuffleboardLayout robotCommands1 = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot1", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        robotCommands1.add("TurnTo 0", new TurnToAngle(drive, 0));
                        robotCommands1.add("TurnTo 45", new TurnToAngle(drive, 45));
                        robotCommands1.add("Turn To 90", new TurnToAngle(drive, 90));
                        robotCommands1.add("TurnTo 180", new TurnToAngle(drive, 180));
                        robotCommands1.add("TurnTo -45", new TurnToAngle(drive, -45));
                        robotCommands1.add("Turn To -90", new TurnToAngle(drive, -90));
                        robotCommands1.add("Turn To LHideOpp", new TurnToAngle(drive, FieldMap.leftStartHideAngle));                     

    
                        ShuffleboardLayout robotValues = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("RobotValues", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        robotValues.addNumber("LeftMeters", () -> drive.getLeftDistance());
                        robotValues.addNumber("RightMeters", () -> drive.getRightDistance());
                        robotValues.addNumber("LeftVelMPS", () -> drive.getLeftRate());
                        robotValues.addNumber("RightVelMPS", () -> drive.getRightRate());
                        robotValues.addNumber("LeftOut", () -> drive.getLeftOut());
                        robotValues.addNumber("RightOut", () -> drive.getRightOut());
                        robotValues.addNumber("LeftAmps", () -> drive.getLeftAmps());
                        robotValues.addNumber("RightAmps", () -> drive.getRightAmps());
                        robotValues.addNumber("Gyro Yaw", () -> drive.getYaw());
                        robotValues.addNumber("Faults", () -> drive.getFaults());
                        robotValues.addNumber("Target", () -> drive.leftTargetPosition);
                        robotValues.addNumber("Heading", () -> drive.getHeading());
                        robotValues.addNumber("Rotation", () -> drive.getRotationDegrees());
                        ShuffleboardLayout robotValues2 = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(6, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels

                        robotValues2.addBoolean("TuneOn", () -> (drive.tuneOn && drive.lastTuneOn));
                        robotValues2.addBoolean("Left1Connected  (2)", () -> drive.leftLeadConnected);
                        robotValues2.addBoolean("Left2Connected (3)", () -> drive.leftFollowerConnected);
                        robotValues2.addBoolean("Right1Connected (4)", () -> drive.rightLeadConnected);
                        robotValues2.addBoolean("Right2Connected  (5)", () -> drive.rightFollowerConnected);
                        robotValues2.addBoolean("LInPosition", () -> drive.getInPositionLeft());
                        robotValues2.addBoolean("RInPosition", () -> drive.getInPositionRight());
                        robotValues2.addBoolean("LFoll", () -> drive.getLeftFollower());
                        robotValues2.addBoolean("RFoll", () -> drive.getRightFollower());

                        ShuffleboardLayout robotGains = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Gains", BuiltInLayouts.kGrid).withPosition(8, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        robotGains.addNumber("kP", () -> drive.kP);
                        robotGains.addNumber("kI", () -> drive.kI);
                        robotGains.addNumber("kD", () -> drive.kD);

                        robotGains.addNumber("kPturn", () -> drive.kTurnP);
                        robotGains.addNumber("kIturn", () -> drive.kTurnI);
                        robotGains.addNumber("kDturn", () -> drive.kTurnD);

                }

                if (showSubsystems) {

                        ShuffleboardLayout subSystems = Shuffleboard.getTab("CanBus")
                                        .getLayout("Subs 1", BuiltInLayouts.kList).withPosition(0, 2)
                                        .withSize(3, 3).withProperties(Map.of("Label position", "LEFT")); //

                        subSystems.add("Drive", drive);
                       
                        ShuffleboardLayout subSystems1 = Shuffleboard.getTab("CanBus")
                                        .getLayout("Subs 2", BuiltInLayouts.kList).withPosition(3, 2)
                                        .withSize(3, 3).withProperties(Map.of("Label position", "LEFT")); //

                      
                        ShuffleboardLayout scheduler = Shuffleboard.getTab("CanBus")
                                        .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(10, 2).withProperties(Map.of("Label position", "TOP")); //

                        scheduler.add("Scheduler", CommandScheduler.getInstance());

                        ShuffleboardLayout canBus = Shuffleboard.getTab("CanBus")
                                        .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(6, 2)
                                        .withSize(4, 2).withProperties(Map.of("Label position", "TOP")); // labels

                       
                }
        }

}