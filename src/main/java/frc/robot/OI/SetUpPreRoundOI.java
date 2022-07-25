// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.Common.HideIt;
import frc.robot.commands.RobotDrive.SetRobotPose;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
public class SetUpPreRoundOI implements Loggable {
        @Log
        public SendableChooser<Integer> autoChooser = new SendableChooser<>();
        @Log
        public SendableChooser<Double> startDelayChooser = new SendableChooser<>();
        @Log
        public SendableChooser<Boolean> hideCargoChooser = new SendableChooser<>();

        public static boolean m_showPreRound;
        double rate = .5;

        public SetUpPreRoundOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight ll, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory fftraj) {

                /**
                 * 
                 * Pre round
                 */

                // Put
                // autonomous chooser on the dashboard.
                // The first argument is the root container
                // The second argument is whether logging and config should be given separate
                // tabs

                autoChooser.setDefaultOption("Do Nothing", 0);

                autoChooser.addOption("Taxi", 1);

                autoChooser.addOption("Left Tarmac Retract Pickup Shoot",
                                2);

                autoChooser.addOption("Center Tarmac Retract Pickup Shoot",
                                3);

                autoChooser.addOption("Center Tarmac Retract Pickup Shoot + 3 & 4 Cargo",
                                4);

                autoChooser.addOption("Right Tarmac Retract Pickup + 3 & 4 Shoot Cargo",
                                5);

                autoChooser.addOption("Retract Pickp Opp Shoot One Hide",
                                6);

                hideCargoChooser.setDefaultOption("No", false);
                hideCargoChooser.addOption("Yes", true);

                startDelayChooser.setDefaultOption("No Delay", 0.);
                startDelayChooser.addOption("One Second", 1.);
                startDelayChooser.addOption("Two Seconds", 2.);
                startDelayChooser.addOption("Three Seconds", 3.);
                startDelayChooser.addOption("Four Seconds", 4.);
                startDelayChooser.addOption("Five Seconds", 5.);


                ("Turn To LHideOpp", new TurnToAngle(drive, FieldMap.leftStartHideAngle));                     

        }
}
