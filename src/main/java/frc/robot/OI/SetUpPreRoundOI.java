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
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(4, 1)
                                        .withPosition(0, 0); // place it in the top-left corner

                        autoChooser.setDefaultOption("Do Nothing", 0);

                        autoChooser.addOption("Taxi", 1);

                        autoChooser.addOption("Left Tarmac Retract Pickup Ret'n Shoot 2",
                                        2);

                        autoChooser.addOption("Right Tarmac Edge Retract Pickup Ret'n Shoot 2",
                                        3);

                        autoChooser.addOption("Right Tarmac Center Retract Pickup Ret'n Shoot 2",
                                        4);

                        autoChooser.addOption("Shoot 1 from under Hub then Retract",
                                        5);

                        autoChooser.addOption("Retract to Tarmac Line, Shoot 1",
                                        6);

                        autoChooser.addOption("Left Tarmac, Retract, Pickup, Ret'n to Tarmac Line, Shoot 2",
                                        7);

                        autoChooser.addOption("Center 3 Ball", 8);

                        Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1)
                                        .withPosition(4, 0); //

                        startDelayChooser.setDefaultOption("No Delay", 0.);
                        startDelayChooser.addOption("One Second", 1.);
                        startDelayChooser.addOption("Two Seconds", 2.);
                        startDelayChooser.addOption("Three Seconds", 3.);
                        startDelayChooser.addOption("Four Seconds", 4.);
                        startDelayChooser.addOption("Five Seconds", 5.);

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiStates", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5)
                                        .withProperties(Map.of("Label position", "TOP"));

                        tiltValues2.addBoolean("TiInPosition", () -> tilt.atTargetAngle());

                        tiltValues2.addBoolean("TiOnBottomLS", () -> tilt.m_reverseLimit.isPressed());

                        tiltValues2.addBoolean("TiOnTopLS", () -> tilt.m_forwardLimit.isPressed());

                        tiltValues2.addBoolean("TiPosResetDone", () -> tilt.positionResetDone);

                        tiltValues2.addBoolean("Ti+SWLimit", () -> tilt.onPlusSoftwareLimit());

                        tiltValues2.addBoolean("Ti-SWLimit", () -> tilt.onMinusSoftwareLimit());

                        tiltValues2.addBoolean("TiSWLimitEn", () -> tilt.getSoftwareLimitsEnabled());

                        ShuffleboardLayout tiltValues3 = Shuffleboard.getTab("SetupTilt")
                        .getLayout("TuStates", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 3)
                        .withProperties(Map.of("Label position", "TOP"));


                        tiltValues3.addBoolean("TuPlusLimit", () -> turret.onPlusSoftwareLimit());

                        tiltValues3.addBoolean("TuMinusLimit", () -> turret.onMinusSoftwareLimit());

                        tiltValues3.addBoolean("TuSWLimitEn", () -> turret.getSoftwareLimitsEnabled());

                }
        }
}