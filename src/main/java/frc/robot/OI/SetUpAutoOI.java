// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.LRetPuShoot;
import frc.robot.commands.AutoCommands.LRetPuShootLow;
import frc.robot.commands.AutoCommands.RRetCenPuShoot;
import frc.robot.commands.AutoCommands.RRetPuS3;
import frc.robot.commands.AutoCommands.RRetPuShoot;
import frc.robot.commands.AutoCommands.RRetPuShootLow;
import frc.robot.commands.AutoCommands.Taxi;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/** Add your docs here. */
public class SetUpAutoOI {

        private final RevTiltSubsystem m_tilt;
        private final RevTurretSubsystem m_turret;
        private final RevDrivetrain m_robotDrive;
        private final RevShooterSubsystem m_shooter;
        private final CargoTransportSubsystem m_transport;
        private final Compressor m_compressor;
        private final LimeLight m_limelight;
        private final IntakesSubsystem m_intake;
        private final ClimberSubsystem m_climber;
        public SendableChooser<Command> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
        private boolean m_showAuto;
        private RawContoursV2 m_rcv2;
        private GetTarget m_target;

        public SetUpAutoOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory traj, RawContoursV2 rcv2, GetTarget target, boolean liveMatch) {
                m_turret = turret;
                m_tilt = tilt;
                m_robotDrive = drive;
                m_transport = transport;
                m_compressor = compressor;
                m_shooter = shooter;
                m_limelight = limelight;
                m_intake = intake;
                m_climber = climber;
                m_rcv2 = rcv2;
                m_target = target;
                /**
                 * 
                 * Pre round
                 */

                if (m_showAuto) {
                        // Put
                        // autonomous chooser on the dashboard.
                        // The first argument is the root container
                        // The second argument is whether logging and config should be given separate
                        // tabs
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1)
                                        .withPosition(0, 0); // place it in the top-left corner
double distance=0;
                        autoChooser.setDefaultOption("Taxi", new Taxi(drive, distance));

                        autoChooser.addOption("Left Tarmac Retract Pickup Shoot",
                                        new LRetPuShoot(intake, drive, turret, tilt, limelight, shooter, m_rcv2,
                                                        m_target, m_transport, m_compressor));

                        autoChooser.addOption("Right Tarmac Edge Retract Pickup Shoot", new RRetPuShoot());
  
                        autoChooser.addOption("Right Tarmac Center Retract Pickup Shoot", new RRetCenPuShoot());

                        autoChooser.addOption("Right Tarmac Shoot 3", new RRetPuS3());

                        autoChooser.addOption("Right Tarmac Shoot Low", new RRetPuShootLow());

                        autoChooser.addOption("Left Tarmac Shoot Low", new LRetPuShootLow(intake, drive, turret, tilt));

                        Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1)
                                        .withPosition(2, 0); //

                        startDelayChooser.setDefaultOption("No Delay", 0);
                        startDelayChooser.addOption("One Second", 1);
                        startDelayChooser.addOption("Two Seconds", 2);
                        startDelayChooser.addOption("Three Seconds", 3);
                        startDelayChooser.addOption("Four Seconds", 4);
                        startDelayChooser.addOption("Five Seconds", 5);
                }

        }
}