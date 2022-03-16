// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.AllRetPuShoot;
import frc.robot.commands.AutoCommands.DoNothing;
import frc.robot.commands.AutoCommands.Common.LeftRetPuAdv;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/** Add your docs here. */
public class SetUpPreRoundOI {

        public SendableChooser<Command> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
        public static boolean m_showPreRound;
        double rate = .5;

        public SetUpPreRoundOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight ll, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory traj, RawContoursV2 rcv2) {

                /**
                 * 
                 * Pre round
                 */

                if (m_showPreRound) {
                        // Put
                        // autonomous chooser on the dashboard.
                        // The first argument is the root container
                        // The second argument is whether logging and config should be given separate
                        // tabs
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1)
                                        .withPosition(0, 0); // place it in the top-left corner
                        double distance = 0;
                        double[] data = { 0, 0, 0, 0, 0, 0, 0 };

                        autoChooser.setDefaultOption("Do Nothing", new DoNothing());

                        autoChooser.addOption("Taxi", new PositionStraight(drive, distance, rate));

                        autoChooser.addOption("Left Tarmac Retract Pickup Advance Shoot",
                                        new LeftRetPuAdv(intake, drive, transport, shooter));

                        autoChooser.addOption("Right Tarmac Edge Retract Pickup Shoot",
                                        new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2,
                                                        transport, compressor, data));

                        autoChooser.addOption("Right Tarmac Center Retract Pickup Shoot",
                                        new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2,
                                                        transport, compressor, data));

                        // autoChooser.addOption("Right Tarmac Shoot 3", new RRetPuS3());

                        Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1)
                                        .withPosition(2, 0); //

                        startDelayChooser.setDefaultOption("No Delay", 0);
                        startDelayChooser.addOption("One Second", 1);
                        startDelayChooser.addOption("Two Seconds", 2);
                        startDelayChooser.addOption("Three Seconds", 3);
                        startDelayChooser.addOption("Four Seconds", 4);
                        startDelayChooser.addOption("Five Seconds", 5);

                        HttpCamera llght = new HttpCamera("CoprocessorCamera",
                                        "http://10.21.94.11:5800/stream.mjpg");

                        ShuffleboardTab llFeed = Shuffleboard.getTab("Pre-Round");

                        llFeed.add("Limelight_90", llght).withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(5, 0).withSize(3, 3)
                                        .withProperties(Map.of("Show Crosshair", true,
                                                        "Show Controls", true, "Rotation", "QUARTER_CW"));

                }

        }
}