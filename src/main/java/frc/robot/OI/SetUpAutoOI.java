// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
    public SendableChooser<Integer> autoChooser = new SendableChooser<>();
    public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();

    public SetUpAutoOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
            RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
            LimeLight limelight, IntakesSubsystem intake, ClimberSubsystem climber,
            FondyFireTrajectory traj, boolean liveMatch) {
        m_turret = turret;
        m_tilt = tilt;
        m_robotDrive = drive;
        m_transport = transport;
        m_compressor = compressor;
        m_shooter = shooter;
        m_limelight = limelight;
        m_intake = intake;
        m_climber = climber;
        /**
         * 
         * Pre round
         */

        if (true) {
            // Put
            // autonomous chooser on the dashboard.
            // The first argument is the root container
            // The second argument is whether logging and config should be given separate
            // tabs
            Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1)
                    .withPosition(0, 0); // place it in the top-left corner

            autoChooser.setDefaultOption("Cross Line", 0);

            autoChooser.addOption("Center Start Retract Shoot", 1);

            autoChooser.addOption("Trench 3 M 2", 2);

            autoChooser.addOption("ShieldGen 3 M 1", 3);

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