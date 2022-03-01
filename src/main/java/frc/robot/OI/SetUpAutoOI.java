// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldMap;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.AllRetPuShoot;
import frc.robot.commands.AutoCommands.DoNothing;
import frc.robot.commands.AutoCommands.Taxi;
import frc.robot.commands.CargoTransport.HoldCargo;
import frc.robot.commands.CargoTransport.ReleaseCargo;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.UseVision;
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

        public SendableChooser<Command> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
        private boolean showAuto;
        private HttpCamera LLFeed;

        public SetUpAutoOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight ll, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory traj, RawContoursV2 rcv2,  boolean liveMatch) {

                /**
                 * 
                 * Pre round
                 */

                if (showAuto || liveMatch) {
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

                        autoChooser.addOption("Taxi", new Taxi(drive, distance));

                        autoChooser.addOption("Left Tarmac Retract Pickup Shoot",
                                        new AllRetPuShoot(intake, drive, turret, tilt, ll, shooter, rcv2,
                                                         transport, compressor, data));

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

                        ShuffleboardLayout competition = Shuffleboard.getTab("Competition")
                                        .getLayout("ShootConditions", BuiltInLayouts.kGrid).withPosition(1, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP"));

                        competition.addBoolean("TiltOnTarget",
                                        () -> ll.getVertOnTarget(tilt.tiltVisionTolerance));
                        competition.addBoolean("TurretOnTarget",
                                        () -> ll.getHorOnTarget(turret.turretVisionTolerance));
                        competition.addBoolean("ShooterAtSpeed", () -> shooter.atSpeed());
                        competition.addBoolean("Use Vision", () -> ll.useVision);
                        competition.addBoolean("RollersAtSpeed", () -> transport.rollersAtSpeed);

                        if (RobotBase.isReal()) {

                                LLFeed = new HttpCamera("Limelight", "http://limelight.local:5800/stream.mjpg");
                                ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("Competition");
                                driverDisplayTab.add("Limelight", LLFeed).withWidget(BuiltInWidgets.kCameraStream)
                                                .withPosition(3, 0).withSize(6, 5)
                                                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify
                                                                                                                        // widget
                                                                                                                        // properties
                        }

                }

                ShuffleboardLayout miscComp = Shuffleboard.getTab("CompetitionMisc")
                                .getLayout("Misc1", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                miscComp.add("Reset to 0", new ResetTurretAngle(turret));
                miscComp.addNumber("TUAngle", () -> turret.getAngle());
                miscComp.addNumber("TiltAngle", () -> tilt.getAngle());
                miscComp.addNumber("RPM", () -> shooter.getRPM());
                miscComp.addNumber("LeftAmps", () -> shooter.getLeftAmps());
                miscComp.addNumber("RightAmps", () -> shooter.getRightAmps());
                miscComp.addNumber("TargetArea%Scrn", () -> ll.getTargetArea());
                miscComp.addNumber("BNDBoxWidth", () -> ll.getBoundingBoxWidth());
                miscComp.addNumber("BndBoxHeight", () -> ll.getBoundingBoxHeight());
                miscComp.addNumber("AspectRatio", () -> ll.getAspectRatio());

                miscComp.addNumber("RQDRPM", () -> shooter.requiredRPM);

                ShuffleboardLayout misComp1 = Shuffleboard.getTab("CompetitionMisc")
                                .getLayout("Misc2", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                misComp1.add("Hold Cargo", new HoldCargo(transport));

                misComp1.add("Release Cargo", new ReleaseCargo(transport));

             //   misComp1.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());

                ShuffleboardLayout misComp2 = Shuffleboard.getTab("CompetitionMisc")
                                .getLayout("Misc3", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                misComp2.addNumber("TargetDistance", () -> shooter.calculatedCameraDistance);
                misComp2.addNumber("CameraCalcSpeed", () -> shooter.cameraCalculatedSpeed);
                misComp2.addNumber("CameraCalcTilt", () -> tilt.cameraCalculatedTiltOffset);
                misComp2.add("Reset Enc", new ResetEncoders(drive));
                misComp2.add("Reset Gyro", new ResetGyro(drive));
                misComp2.addNumber("LeftMeters", () -> drive.getLeftDistance());
                misComp2.addNumber("RightMeters", () -> drive.getRightDistance());

                ShuffleboardLayout misComp3 = Shuffleboard.getTab("CompetitionMisc")
                                .getLayout("Misc4", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));

                misComp3.addBoolean("RobotStopped", () -> drive.robotStoppedForOneSecond);
                //misComp3.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());
                misComp3.addBoolean("No CargoAtShoot", () -> transport.noCargoAtShooterForOneSecond);

                misComp3.addBoolean("CargoAvailable", () -> transport.cargoAvailable);
                
                misComp3.addBoolean("IsShooting", () -> shooter.isShooting);

                ShuffleboardLayout misComVis = Shuffleboard.getTab("CompetitionMisc")
                                .getLayout("MiscVis", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4)
                                .withProperties(Map.of("Label position", "LEFT"));
                misComVis.add("No Zoom Pipeline", new LimelightSetPipeline(ll, 1));
                misComVis.add("Driver Pipeline", new LimelightSetPipeline(ll, 0));
                misComVis.add("Vision On", new UseVision(ll, true));
                misComVis.add("Vision Off", new UseVision(ll, false));
                misComVis.addBoolean("LLTGT", () -> ll.getIsTargetFound());
                misComVis.addBoolean("TiVT", () -> tilt.validTargetSeen);
                misComVis.addBoolean("TuVT", () -> turret.validTargetSeen);

                // Shuffleboard.getTab("Competition").addNumber("TimeRemaining", () -> drive.getMatchTime())
                //                 .withWidget(BuiltInWidgets.kTextView).withPosition(9, 0).withSize(1, 1);
                // Shuffleboard.getTab("Competition").addNumber("Battery", () -> getPDPInfo()[0])
                //                 .withWidget(BuiltInWidgets.kTextView).withPosition(9, 1).withSize(1, 1);
                // Shuffleboard.getTab("Competition").addNumber("TotalEnegy Ah", () -> getPDPInfo()[2])
                //                 .withWidget(BuiltInWidgets.kTextView).withPosition(9, 2).withSize(1, 1);

        }

}
