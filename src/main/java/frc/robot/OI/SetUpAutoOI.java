// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotBase;
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

        public SetUpAutoOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight ll, IntakesSubsystem intake, ClimberSubsystem climber,
                        FondyFireTrajectory traj) {

                if (m_showAuto) {

                        ShuffleboardLayout compet = Shuffleboard.getTab("Competition")
                                        .getLayout("States", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "TOP"));
                        
         
                        compet.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());
                        compet.addBoolean("IsShooting", () -> shooter.isShooting);
                        compet.addBoolean("CargoAtRear", () -> intake.getCargoAtRear());
                        compet.addBoolean("CargoAtFront", () -> intake.getCargoAtFront());
                        compet.addBoolean("TiltOnSwitch", () -> tilt.onMinusHardwareLimit());
                        compet.addBoolean("LockAtTarget", () -> turret.getLockAtTarget());
                        ShuffleboardLayout compet1 = Shuffleboard.getTab("Competition")
                                        .getLayout("Values", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet1.addNumber("TUAngle", () -> turret.getAngle());
                        compet1.addNumber("TiltAngle", () -> tilt.getAngle());
                        compet1.addNumber("ShooterRPM", () -> shooter.getRPM());
                        compet1.addNumber("LeftAmps", () -> shooter.getLeftAmps());
                        compet1.addNumber("RightAmps", () -> shooter.getRightAmps());
                        compet1.addNumber("RobotPosn", () -> drive.getAverageDistance());
                        
                        compet1.addNumber("TargetDistance", () -> shooter.calculatedCameraDistance);

                        compet1.addBoolean("LLHasTarget", () -> ll.getIsTargetFound());
                        ShuffleboardLayout compet2 = Shuffleboard.getTab("Competition")
                                        .getLayout("Colors", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 6)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet2.addBoolean("Blue Alliance", () -> Robot.getAllianceColorBlue());
                        compet2.addBoolean("Red Alliance", () -> !Robot.getAllianceColorBlue());
                        compet2.addBoolean("WrongColorCargo", () -> transport.wrongCargoColor);
                        compet2.addBoolean("BlueCargo", () -> transport.cargoIsBlue);
                        compet2.addBoolean("RedCargo", () -> transport.cargoIsRed);
                        compet2.addBoolean("TwoCargoOnBoard", () -> intake.twoCargoOnBoard);
                        compet2.addBoolean("FrontIntake", () -> intake.useFrontIntake);

                        ShuffleboardLayout compet3 = Shuffleboard.getTab("Competition")
                                        .getLayout("Important", BuiltInLayouts.kList).withPosition(4, 0).withSize(1, 6)
                                        .withProperties(Map.of("Label position", "TOP"));
                        compet3.addNumber("BatteryVolts", () -> RobotController.getBatteryVoltage());
                        compet3.addNumber("Match Time", () -> Robot.matchTimeRemaining);

                        ShuffleboardLayout compet5 = Shuffleboard.getTab("Competition")
                                        .getLayout("AutoData", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 6)
                                        .withProperties(Map.of("Label position", "TOP"));

                        compet5.addNumber("AutoRPM", () -> Robot.data[4]);
                        compet5.addNumber("AutoTiltDeg", () -> Robot.data[2]);
                        compet5.addNumber("AutoTurretDeg", () -> Robot.data[3]);
                        compet5.addNumber("RetctPosnInch",
                                        () -> Math.round(Units.metersToInches(Robot.data[0]) * 10) / 10);
                        compet5.addNumber("ShootPosnInch",
                                        () -> Math.round(Units.metersToInches(Robot.data[1]) * 10)
                                                        / 10);

                        ShuffleboardLayout compet4 = Shuffleboard.getTab("Competition")
                                        .getLayout("PresetData", BuiltInLayouts.kList).withPosition(3, 0).withSize(1, 6)
                                        .withProperties(Map.of("Label position", "TOP"));

                        compet4.addString("PresetLocation", () -> shooter.presetLocationName);
                        compet4.addNumber("PresetRPM", () -> shooter.presetRPM);
                        compet4.addNumber("TiltPreset", () -> tilt.presetPosition);
                        compet4.addNumber("DriverAdjustRPM",
                                        () -> shooter.shooterRPMAdder[shooter.shootLocation]);
                        
                        compet4.addNumber("TUAngle", () -> turret.getAngle());
                        compet4.addNumber("Visiondistance", () -> shooter.calculatedCameraDistance);
                        compet4.addBoolean("LLHasTarget", () -> ll.getIsTargetFound());
                        compet4.addBoolean("WrongColor", () -> transport.wrongCargoColor);

                        ShuffleboardTab llFeed = Shuffleboard.getTab("Competition");

                        if (RobotBase.isReal()) {

                                llFeed.addCamera("LL", "Copro1Cam", "http://10.21.94.11:5800/stream.mjpg")
                                                .withPosition(5, 0).withSize(3, 4)
                                                .withProperties(Map.of("Show Crosshair", true,
                                                                "Show Controls", true, "Rotation", "QUARTER_CW"));
                                if (intake.useFrontCamera) {
                                        UsbCamera frontIntakeCamera = CameraServer.startAutomaticCapture("FrontCam", 0);
                                        frontIntakeCamera.setResolution(320, 240);
                                        frontIntakeCamera.setFPS(20);

                                        ShuffleboardTab frontFeed = Shuffleboard.getTab("FrontIntakeCamera");

                                        frontFeed.add("FrontCamera", frontIntakeCamera)
                                                        .withWidget(BuiltInWidgets.kCameraStream)
                                                        .withPosition(2, 0).withSize(6, 4)
                                                        .withProperties(Map.of("Show Crosshair", false, "Show Controls",
                                                                        true));
                                }

                                if (intake.useRearCamera) {
                                        ShuffleboardTab rearFeed = Shuffleboard.getTab("RearIntakeCamera");

                                        UsbCamera rearIntakeCamera = CameraServer.startAutomaticCapture("RearCam", 1);
                                        rearIntakeCamera.setResolution(320, 240);
                                        rearIntakeCamera.setFPS(20);

                                        rearFeed.add("RearCamera", rearIntakeCamera)
                                                        .withWidget(BuiltInWidgets.kCameraStream)
                                                        .withPosition(2, 0).withSize(6, 4)
                                                        .withProperties(Map.of("Show Crosshair", false, "Show Controls",
                                                                        false));
                                }
                        }
                }

        }
}