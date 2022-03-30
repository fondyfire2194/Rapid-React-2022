// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
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

        public SendableChooser<Integer> autoChooser = new SendableChooser<>();
        public SendableChooser<Double> startDelayChooser = new SendableChooser<>();
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
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(3, 1)
                                        .withPosition(0, 0); // place it in the top-left corner

                        autoChooser.setDefaultOption("Do Nothing", 0);

                        autoChooser.addOption("Taxi", 1);

                        autoChooser.addOption("Left Tarmac Retract Pickup Advance Shoot",
                                        2);

                        autoChooser.addOption("Right Tarmac Edge Retract Pickup Shoot",
                                        3);

                        autoChooser.addOption("Right Tarmac Center Retract Pickup Shoot",
                                        4);

                        autoChooser.addOption("Shoot from under Hub then Retract",
                                        5);

                        autoChooser.addOption("Left Tarmac, Retract, Pickup, to Tarmac Line, Shoot",
                                        6);

                        autoChooser.addOption("Center 3 Ball", 7);

                        Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1)
                                        .withPosition(3, 0); //

                        startDelayChooser.setDefaultOption("No Delay", 0.);
                        startDelayChooser.addOption("One Second", 1.);
                        startDelayChooser.addOption("Two Seconds", 2.);
                        startDelayChooser.addOption("Three Seconds", 3.);
                        startDelayChooser.addOption("Four Seconds", 4.);
                        startDelayChooser.addOption("Five Seconds", 5.);


                        ShuffleboardLayout checkStuff = Shuffleboard.getTab("Pre-Round")
                        .getLayout("While There's Still Time", BuiltInLayouts.kList).withPosition(3,2).withSize(3,1)
                        .withProperties(Map.of("Label position", "TOP"));

                        checkStuff.addString("Message from Mark", ()->"Has Everything Been Checked - Twice?");

                }

        }
}