// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldMap;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.CargoTransport.RunLowerRoller;
import frc.robot.commands.CargoTransport.StopLowerRoller;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Intakes.StopIntakeMotors;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.SaveGetSavedPose;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.RunTopRoller;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Shooter.StopTopRoller;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.PositionTiltToEntry;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Turret.StopTurret;
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

        public SetUpOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, IntakesSubsystem intake,
                        ClimberSubsystem climber,
                        FondyFireTrajectory traj) {

                if (showIntake) {

                        ShuffleboardLayout intakeSelect = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeSelect", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeSelect.add("Select Front", new SetFrontIntakeActive(intake, true));
                        intakeSelect.add("Select Rear", new SetFrontIntakeActive(intake, false));
                        intakeSelect.addBoolean("RearActive", () -> !intake.useFrontIntake);
                        intakeSelect.addBoolean("FrontActive", () -> intake.useFrontIntake);
                        intakeSelect.addBoolean("FrontConnected", () -> intake.frontIntakeMotorConnected);
                        intakeSelect.addBoolean("RearConnected", () -> intake.rearIntakeMotorConnected);

                        ShuffleboardLayout intakeActions = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeActions", BuiltInLayouts.kList).withPosition(1, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeActions.add("ArmRaise", new ActiveIntakeArmRaise(intake));
                        intakeActions.add("ArmLower", new ActiveIntakeArmLower(intake));
                        intakeActions.add("Run Motor", new RunActiveIntake(intake, transport));
                        intakeActions.add("Stop Motor", new StopIntakeMotors(intake));

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeValues.addBoolean("Arm Up", () -> intake.getActiveArmRaised());
                        intakeValues.addBoolean("Arm Down", () -> intake.getActiveArmLowered());
                        intakeValues.addNumber("Motor Amps", () -> intake.getActiveMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> intake.getActiveMotor());
                        intakeValues.addBoolean("CargoAtRearIn", () -> intake.getCargoAtRear());
                        intakeValues.addBoolean("CargoAtFrontIn", () -> intake.getCargoAtFront());
                        intakeActions.add("IntakeCmd", intake);

                        if (intake.useFrontCamera) {
                                UsbCamera frontIntakeCamera = CameraServer.startAutomaticCapture("FrontCam", 0);

                                ShuffleboardTab frontFeed = Shuffleboard.getTab("FrontIntakeCamera");

                                frontFeed.add("FrontCamera", frontIntakeCamera).withWidget(BuiltInWidgets.kCameraStream)
                                                .withPosition(2, 0).withSize(6, 4)
                                                .withProperties(Map.of("Show Crosshair", false, "Show Controls", true));
                        }

                        if (intake.useRearCamera) {

                                ShuffleboardTab rearFeed = Shuffleboard.getTab("RearIntakeCamera");

                                UsbCamera rearIntakeCamera = CameraServer.startAutomaticCapture("RearCam", 1);

                                rearFeed.add("RearCamera", rearIntakeCamera).withWidget(BuiltInWidgets.kCameraStream)
                                                .withPosition(2, 0).withSize(6, 4)
                                                .withProperties(Map.of("Show Crosshair", false, "Show Controls",
                                                                false));

                        }
                }

                if (showTurret) {

                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretCommands.add("Reset to 0", new ResetTurretAngle(turret));
                        turretCommands.add("Position To 0", new PositionTurret(turret, 0));//
                        // degrees
                        turretCommands.add("Position To -20", new PositionTurret(turret, -20));//

                        turretCommands.add("Position To 20", new PositionTurret(turret, 20));

                        turretCommands.add("StopTurret", new StopTurret(turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(turret));
                        turretCommands.add("Cmd", turret);
                        turretCommands.addNumber("Faults", () -> turret.getFaults());

                        turretCommands.add("Vision On", new UseVision(limelight, true));
                        turretCommands.add("Vision Off", new UseVision(limelight, false));

                        ShuffleboardLayout turretValues = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("TurretValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        turretValues.addNumber("TUAngle", () -> turret.getAngle());
                        turretValues.addNumber("TUTgt", () -> turret.targetAngle);
                        turretValues.addNumber("Pct", () -> turret.getMotorOut());
                        turretValues.addNumber("Amps", () -> turret.getAmps());
                        turretValues.addNumber("Speed", () -> turret.getSpeed());
                        turretValues.addNumber("Vision Error", () -> limelight.getdegRotationToTarget());
                        turretValues.addBoolean("MinHWLim", () -> turret.onMinusHardwareLimit());
                        turretValues.addBoolean("PlusHWLim", () -> turret.onPlusHardwareLimit());

                        ShuffleboardLayout turretValues3 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2,
                                                        2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretValues3.addNumber("IAccum", () -> turret.getIaccum());

                        turretValues3.addNumber("LockOutput", () -> turret.lockPIDOut);
                        turretValues3.addNumber("LockError", () -> turret.mLockController.getPositionError());
                        turretValues3.addBoolean("LockController", () -> turret.validTargetSeen);

                        ShuffleboardLayout turretValues2 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 2).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels
                        turretValues2.addBoolean("Connected (8)", () -> turret.turretMotorConnected);

                        turretValues2.addBoolean("PlusLimit", () -> turret.onPlusSoftwareLimit());

                        turretValues2.addBoolean("MinusLimit", () -> turret.onMinusSoftwareLimit());

                        turretValues2.addBoolean("SWLimitEn", () -> turret.getSoftwareLimitsEnabled());

                        turretValues2.addBoolean("InPosition", () -> turret.atTargetAngle());

                        turretValues2.addBoolean("TargetHorOK",
                                        () -> limelight.getHorOnTarget(turret.turretVisionTolerance));

                        turretValues2.addBoolean("OKTune", () -> (turret.tuneOnv &&
                                        turret.lastTuneOnv));

                        turretValues2.addBoolean("LockAtTarget", () -> turret.getLockAtTarget());

                        ShuffleboardLayout turretVactGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("VActGains", BuiltInLayouts.kList).withPosition(5, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        turretVactGains.addNumber("FF", () -> turret.ffsetv);
                        turretVactGains.addNumber("P", () -> turret.psetv);
                        turretVactGains.addNumber("D", () -> turret.dsetv);
                        turretVactGains.addNumber("I", () -> turret.isetv);
                        turretVactGains.addNumber("IZ", () -> turret.izsetv);
                        turretVactGains.addNumber("maxVel", () -> turret.maxVel);

                        ShuffleboardLayout turretLockGains = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("LockGains", BuiltInLayouts.kList)
                                        .withPosition(6, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        turretLockGains.addNumber("tuLkp", () -> turret.mLockController.getP());
                        turretLockGains.addNumber("tuLki", () -> turret.mLockController.getI());
                        turretLockGains.addNumber("tuLkd", () -> turret.mLockController.getD());
                        turretLockGains.addNumber("tuLPosErr", () -> turret.mLockController.getPositionError());

                        ShuffleboardLayout turretPosGains = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("PosGains", BuiltInLayouts.kList)
                                        .withPosition(7, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        turretPosGains.addNumber("tuPkp", () -> turret.mPosController.getP());
                        turretPosGains.addNumber("tuPki", () -> turret.mPosController.getI());
                        turretPosGains.addNumber("tuPkd", () -> turret.mPosController.getD());
                        turretPosGains.addNumber("tuPPoseErr", () -> turret.mPosController.getPositionError());

                }

                if (showTilt) {

                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Position To Entry", new PositionTiltToEntry(tilt));
                        tiltCommands.add("PositionToSwitch", new TiltMoveToReverseLimit(tilt));
                        tiltCommands.add("StopTilt", new StopTilt(tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(tilt));
                        tiltCommands.add("Cmd", tilt);
                        tiltCommands.addNumber("Faults", () -> tilt.faultSeen);

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TIAngle", () -> tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> tilt.targetAngle);
                        tiltValues.addNumber("PresetAngle", () -> tilt.presetPosition);
                        tiltValues.addNumber("PCT", () -> tilt.getMotorOut());
                        tiltValues.addNumber("Amps", () -> tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> tilt.getSpeed());
                        tiltValues.addNumber("AdjTarget", () -> tilt.adjustedVerticalError);
                        tiltValues.addNumber("VertDegToTarget", () -> limelight.getdegVerticalToTarget());

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP"));

                        tiltValues2.addBoolean("InPosition", () -> tilt.atTargetAngle());

                        tiltValues2.addBoolean("OnBottomLS", () -> tilt.m_reverseLimit.isPressed());
                        tiltValues2.addBoolean("OnTopLS", () -> tilt.m_forwardLimit.isPressed());

                        tiltValues2.addBoolean("PosResetDone", () -> tilt.positionResetDone);
                        tiltValues2.addBoolean("OKTune", () -> (tilt.tuneOnv && tilt.lastTuneOnv));
                        tiltValues2.addBoolean("Connected (9)", () -> tilt.tiltMotorConnected);
                        tiltValues2.addBoolean("+SWLimit", () -> tilt.onPlusSoftwareLimit());
                        tiltValues2.addBoolean("-SWLimit", () -> tilt.onMinusSoftwareLimit());
                        tiltValues2.addBoolean("SWLimitEn", () -> tilt.getSoftwareLimitsEnabled());

                        ShuffleboardLayout tiltVactGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("VActGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        tiltVactGains.addNumber("FF", () -> tilt.ffsetv);
                        tiltVactGains.addNumber("P", () -> tilt.psetv);
                        tiltVactGains.addNumber("D", () -> tilt.dsetv);
                        tiltVactGains.addNumber("I", () -> tilt.isetv);
                        tiltVactGains.addNumber("IZ", () -> tilt.izsetv);

                        ShuffleboardLayout tiltPosGains = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("PosGains", BuiltInLayouts.kList)
                                        .withPosition(8, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        tiltPosGains.addNumber("tiPkp", () -> tilt.mPosController.getP());
                        tiltPosGains.addNumber("tiPki", () -> tilt.mPosController.getI());
                        tiltPosGains.addNumber("tiPkd", () -> tilt.mPosController.getD());

                }

                if (showShooter) {

                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("MAXRPM 5500", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        shooterCommands.add("Stop Shoot", new StopShoot(shooter, transport));

                        shooterCommands.add("Start Shooter", new RunShooter(shooter));

                        shooterCommands.add("ShootOne",
                                        new AltShootCargo(shooter, transport, intake, limelight));
                        shooterCommands.add("ShootTwo",
                                        new AltShootCargo(shooter, transport, intake, limelight));

                        shooterCommands.add("ClearFaults", new ClearShFaults(shooter));

                        shooterCommands.add("RunShooterFromThrottle",
                                        new SetShootSpeedSource(shooter, 0));

                        shooterCommands.add("RunTopRoll", new RunTopRoller(shooter, 750));

                        shooterCommands.add("StopTopRoll", new StopTopRoller(shooter));

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues.addNumber("ActualRPM", () -> shooter.getRPM());
                        shooterValues.addNumber("Required RPM", () -> shooter.getRPMFromSpeedSource());

                        shooterValues.addNumber("TopRollerRPM", () -> shooter.getTopRPM());
                        shooterValues.addNumber("TopTargetRPM", () -> shooter.topRequiredRPM);
                        shooterValues.addNumber("TopRollerAmp", () -> shooter.getTopRollerMotorAmps());
                        shooterValues.addNumber("TopRollerOut", () -> shooter.getTopRoller());

                        shooterValues.addNumber("Left PCT", () -> shooter.getLeftPctOut());
                        shooterValues.addNumber("LeftAmps", () -> shooter.getLeftAmps());

                        shooterValues.addNumber("RightAmps", () -> shooter.getRightAmps());
                        shooterValues.addNumber("LeftFaults", () -> shooter.getLeftFaults());
                        shooterValues.addNumber("RightFaults", () -> shooter.getRightFaults());

                        ShuffleboardLayout shooterValues1 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("ShooterValues1", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        shooterValues1.addBoolean("ShooterAtSpeed", () -> shooter.getShooterAtSpeed());
                        shooterValues1.addBoolean("TopRollerAtSpeed", () -> shooter.getTopRollerAtSpeed());
                        shooterValues.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());
                        shooterValues1.addBoolean("TuneOn", () -> (shooter.tuneOn && shooter.lastTuneOn));
                        shooterValues1.addBoolean("BothConnected(6,7)", () -> shooter.allConnected);
                        shooterValues1.add("Cmd", shooter);

                        ShuffleboardLayout shooterValues3 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("ShooterVlues3", BuiltInLayouts.kList).withPosition(6, 0)
                                        .withSize(2, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels
                        shooterValues3.addString("PresetLocation", () -> shooter.presetLocationName);
                        shooterValues3.addString("ShooterMode", () -> shooter.shootModeName);
                        shooterValues3.addNumber("CalculatedDistance", () -> shooter.calculatedCameraDistance);
                        shooterValues3.addNumber("CalculatedRPM", () -> shooter.cameraCalculatedSpeed);
                        shooterValues3.addNumber("CalculatedTiltAngle", () -> tilt.cameraCalculatedTiltPosition);

                        ShuffleboardLayout shooterValues2 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("Gains", BuiltInLayouts.kList).withPosition(9, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues2.addNumber("FF", () -> shooter.ffset);
                        shooterValues2.addNumber("P", () -> shooter.pset);
                        shooterValues2.addNumber("I", () -> shooter.iset);
                        shooterValues2.addNumber("D", () -> shooter.dset);
                        shooterValues2.addNumber("IZ", () -> shooter.izset);

                }

                if (showTransport) {

                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        transportValues.add("StartLowRoll", new RunLowerRoller(transport, 500));
                        transportValues.add("StopLowRoll", new StopLowerRoller(transport));

                        transportValues.addNumber("LowRollRPM", () -> transport.getLowerRPM());
                        transportValues.addNumber("LowRollCMDRPM", () -> transport.lowerRequiredRPM);
                        transportValues.addNumber("LowRollAmps", () -> transport.getLowerRollerMotorAmps());
                        transportValues.addNumber("LowRollOut", () -> transport.getLowerRoller());

                        transportValues.add("Cmd", transport);
                        transportValues.addBoolean("LowerRollerAtSpeed", () -> transport.getLowerRollerAtSpeed());
                        transportValues.addBoolean("WrongColor", () -> transport.wrongCargoColor);

                        ShuffleboardLayout transportValues1 = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportStates", BuiltInLayouts.kGrid).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // label

                        transportValues1.addBoolean("LowerRollerConnected (14)",
                                        () -> transport.lowerRollerMotorConnected);
                        transportValues1.addBoolean("BlueCargo", () -> transport.cargoIsBlue);
                        transportValues1.addBoolean("RedCargo", () -> transport.cargoIsRed);
                        transportValues1.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());

                }

                if (showClimber) {
                        ShuffleboardLayout climberInfo = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("Climber", BuiltInLayouts.kList).withPosition(7, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        climberInfo.addNumber("ClimberAmps", () -> climber.getMotorAmps());
                        climberInfo.addNumber("ClimberOut", () -> climber.getMotorOut());

                }

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
                        robotCommands.add("To 2", new PositionStraight(drive, 2, .3));
                        robotCommands.add("To -1", new PositionStraight(drive, -1, .3));
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

                        robotCommands1.add("SavePose", new SaveGetSavedPose(drive, 0));
                        robotCommands1.add("SetPose", new SaveGetSavedPose(drive, 3));

                        robotCommands1.add("GetSavedPose", new SaveGetSavedPose(drive, 1));
                        robotCommands1.add("GetCurrentPose", new SaveGetSavedPose(drive, 2));

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
                        subSystems.add("Shooter",
                                        shooter);
                        subSystems.add("Climber", climber);

                        ShuffleboardLayout subSystems1 = Shuffleboard.getTab("CanBus")
                                        .getLayout("Subs 2", BuiltInLayouts.kList).withPosition(3, 2)
                                        .withSize(3, 3).withProperties(Map.of("Label position", "LEFT")); //

                        subSystems1.add("Turret", turret);
                        subSystems1.add("Tilt", tilt);
                        subSystems1.add("Transport", transport);
                        subSystems1.add("Intakes", intake);

                        ShuffleboardLayout scheduler = Shuffleboard.getTab("CanBus")
                                        .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(10, 2).withProperties(Map.of("Label position", "TOP")); //

                        scheduler.add("Scheduler", CommandScheduler.getInstance());

                        ShuffleboardLayout canBus = Shuffleboard.getTab("CanBus")
                                        .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(6, 2)
                                        .withSize(4, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        canBus.addBoolean("TurretConnected (8)", () -> turret.turretMotorConnected);
                        canBus.addBoolean("TiltConnected (9)", () -> tilt.tiltMotorConnected);
                        canBus.addBoolean("ShooterConnected (6,7,10)", () -> shooter.allConnected);

                        canBus.addBoolean("LowerRollerConnected(12)", () -> transport.lowerRollerMotorConnected);
                        canBus.addBoolean("AllDriveConnected  (2,3,4,5)", () -> drive.allConnected);
                        canBus.addBoolean("ClimberConnected(15)", () -> climber.climberMotorConnected);

                        canBus.addBoolean("FrontIntakeConnected (13)", () -> intake.frontIntakeMotorConnected);
                        canBus.addBoolean("RearIntakeConnected (14)", () -> intake.rearIntakeMotorConnected);

                }
        }

}