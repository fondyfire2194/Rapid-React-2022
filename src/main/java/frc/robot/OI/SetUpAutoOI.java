// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Turret.ResetTurretAngle;
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
        public static boolean m_showAuto;
        // double rate = .5;

        public SetUpAutoOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight ll, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory traj, RawContoursV2 rcv2) {

                if (m_showAuto) {
                        // ShuffleboardLayout miscComp = Shuffleboard.getTab("CompetitionMisc")
                        //                 .getLayout("Misc1", BuiltInLayouts.kList).withPosition(0, 3).withSize(4, 4)
                        //                 .withProperties(Map.of("Label position", "LEFT"));

                        // miscComp.add("Reset to 0", new ResetTurretAngle(turret));
                        // miscComp.addNumber("TUAngle", () -> turret.getAngle());
                        // miscComp.addNumber("TiltAngle", () -> tilt.getAngle());
                        // miscComp.addNumber("RPM", () -> shooter.getRPM());
                        // miscComp.addNumber("LeftAmps", () -> shooter.getLeftAmps());
                        // miscComp.addNumber("RightAmps", () -> shooter.getRightAmps());

                        // miscComp.addNumber("RQDRPM", () -> shooter.requiredRPM);

                        // // ShuffleboardLayout misComp1 = Shuffleboard.getTab("CompetitionMisc")
                        // // .getLayout("Misc2", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                        // // .withProperties(Map.of("Label position", "LEFT"));

                        // // misComp1.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());

                        // ShuffleboardLayout misComp2 = Shuffleboard.getTab("CompetitionMisc")
                        //                 .getLayout("Misc3", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4)
                        //                 .withProperties(Map.of("Label position", "LEFT"));

                        // misComp2.addNumber("TargetDistance", () -> shooter.calculatedCameraDistance);
                        // misComp2.addNumber("CameraCalcSpeed", () -> shooter.cameraCalculatedSpeed);
                        // misComp2.addNumber("CameraCalcTilt", () -> tilt.cameraCalculatedTiltPosition);
                        // misComp2.add("Reset Enc", new ResetEncoders(drive));
                        // misComp2.add("Reset Gyro", new ResetGyro(drive));
                        // misComp2.addNumber("LeftMeters", () -> drive.getLeftDistance());
                        // misComp2.addNumber("RightMeters", () -> drive.getRightDistance());

                        // ShuffleboardLayout misComp3 = Shuffleboard.getTab("CompetitionMisc")
                        //                 .getLayout("Misc4", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4)
                        //                 .withProperties(Map.of("Label position", "LEFT"));

                        // misComp3.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());
                        // misComp3.addBoolean("IsShooting", () -> shooter.isShooting);

                        // ShuffleboardLayout misComVis = Shuffleboard.getTab("CompetitionMisc")
                        // .getLayout("MiscVis", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4)
                        // .withProperties(Map.of("Label position", "LEFT"));
                        // misComVis.add("No Zoom Pipeline", new LimelightSetPipeline(ll, 1));
                        // misComVis.add("Driver Pipeline", new LimelightSetPipeline(ll, 0));
                        // misComVis.add("Vision On", new UseVision(ll, true));
                        // misComVis.add("Vision Off", new UseVision(ll, false));
                        // misComVis.addBoolean("LLTGT", () -> ll.getIsTargetFound());
                        // misComVis.addBoolean("TiVT", () -> tilt.validTargetSeen);
                        // misComVis.addBoolean("TuVT", () -> turret.validTargetSeen);

                        ShuffleboardLayout compet = Shuffleboard.getTab("Competition")
                                        .getLayout("States", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));

                        compet.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());
                        compet.addBoolean("IsShooting", () -> shooter.isShooting);
                        compet.addBoolean("CargoAtRear", () -> intake.getCargoAtRear());
                        compet.addBoolean("CargoAtFront", () -> intake.getCargoAtFront());
                        compet.addBoolean("TiltOnSwitch", () -> tilt.onMinusHardwarLimit());
  
                        ShuffleboardLayout compet1 = Shuffleboard.getTab("Competition")
                                        .getLayout("Values", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet1.addNumber("TUAngle", () -> turret.getAngle());
                        compet1.addNumber("TiltAngle", () -> tilt.getAngle());
                        compet1.addNumber("RPM", () -> shooter.getRPM());
                        compet1.addNumber("LeftAmps", () -> shooter.getLeftAmps());
                        compet1.addNumber("RightAmps", () -> shooter.getRightAmps());
                        compet1.addNumber("RQDRPM", () -> shooter.requiredRPM);

                        ShuffleboardLayout compet2 = Shuffleboard.getTab("Competition")
                                        .getLayout("Info", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet2.addBoolean("Blue Alliance", () -> Robot.getAllianceColorBlue());
                        compet2.addBoolean("Red Alliance", () -> !Robot.getAllianceColorBlue());

                        ShuffleboardLayout compet3 = Shuffleboard.getTab("Competition")
                                        .getLayout("Important", BuiltInLayouts.kList).withPosition(3, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet3.addNumber("BatteryVolts", () -> RobotController.getBatteryVoltage());
                        compet3.addNumber("Match Time", () -> DriverStation.getMatchTime());
                        compet3.addBoolean("WrongColorCargo", () -> transport.wrongCargoColor);
                        compet3.addBoolean("TwoCargoOnBoard", () -> intake.twoCargoOnBoard);

                        ShuffleboardTab llFeed = Shuffleboard.getTab("Competition");

                        llFeed.addCamera("LL", "Copro1Cam", "http://10.21.94.11:5800/stream.mjpg")
                                        .withPosition(5, 0).withSize(5, 4)
                                        .withProperties(Map.of("Show Crosshair", true,
                                                        "Show Controls", true, "Rotation", "QUARTER_CW"));

                        // Shuffleboard.getTab("Competition").addNumber("TimeRemaining", () ->
                        // drive.getMatchTime())
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 0).withSize(1, 1);
                        // Shuffleboard.getTab("Competition").addNumber("Battery", () ->
                        // getPDPInfo()[0])
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 1).withSize(1, 1);
                        // Shuffleboard.getTab("Competition").addNumber("TotalEnegy Ah", () ->
                        // getPDPInfo()[2])
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 2).withSize(1, 1);
                }
        }
}