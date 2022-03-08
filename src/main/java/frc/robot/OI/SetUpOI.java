// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.CargoTransport.RunLowerRoller;
import frc.robot.commands.CargoTransport.StopLowerRoller;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntakeMotor;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopIntakeMotors;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.PositionMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Turret.StopTurret;
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

        private boolean showTurret = true;
        private boolean showTilt = true;
        private boolean showShooter = true;
        private boolean showRobot = true;
        private boolean showTransport = true;
        private boolean showClimber = true;
        private boolean showSubsystems = true;
        private boolean showIntake = true;

        public double timeToStart;

        public SetUpOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, IntakesSubsystem intake,
                        ClimberSubsystem climber,
                        FondyFireTrajectory traj, boolean isMatch) {

                if (showIntake && !isMatch) {

                        ShuffleboardLayout intakeSelect = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeSelect", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeSelect.add("Select Front", new SetFrontIntakeActive(intake));
                        intakeSelect.add("Select Rear", new SetRearIntakeActive(intake));
                        intakeSelect.addBoolean("RearActive", () -> !intake.useFrontIntake);
                        intakeSelect.addBoolean("FrontActive", () -> intake.useFrontIntake);
                        intakeSelect.addBoolean("FrontConnected", () -> intake.frontIntakeMotorConnected);
                        intakeSelect.addBoolean("RearConnected", () -> intake.rearIntakeMotorConnected);

                        ShuffleboardLayout intakeActions = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeActions", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP"));

                        intakeActions.add("ArmRaise", new ActiveIntakeArmRaise(intake));
                        intakeActions.add("ArmLower", new ActiveIntakeArmLower(intake));
                        intakeActions.add("Run Motor", new RunActiveIntakeMotor(intake, transport));
                        intakeActions.add("Stop Motor", new StopIntakeMotors(intake));

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(2, 2)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "TOP"));

                        intakeValues.addBoolean("Arm Up", () -> intake.getActiveArmRaised());
                        intakeValues.addBoolean("Arm Down", () -> intake.getActiveArmLowered());
                        intakeValues.addNumber("Motor Amps", () -> intake.getActiveMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> intake.getActiveMotor());
                        intakeValues.addBoolean("CargoAtRearIn", () -> intake.getCargoAtRear());
                        intakeValues.addBoolean("CargoAtFrontIn", () -> intake.getCargoAtFront());

                }

                if (showTurret && !isMatch) {

                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretCommands.add("Reset to 0", new ResetTurretAngle(turret));
                        turretCommands.add("Position To 0", new PositionTurret(turret, 0));//
                        // degrees
                        turretCommands.add("Position To -10", new PositionTurret(turret, -10));//

                        turretCommands.add("Position To 10", new PositionTurret(turret, 10));

                        turretCommands.add("StopTurret", new StopTurret(turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(turret));
                        turretCommands.add("Cmd", turret);
                        turretCommands.addNumber("Faults", () -> turret.getFaults());
                        turretCommands.addString("To Jog", () -> "SetupXBox Btn A left X");
                        turretCommands.addString("OvrRideSoftLim", () -> "Setup RightBmpr");

                        ShuffleboardLayout turretValues = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("TurretValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        turretValues.addNumber("TUAngle", () -> turret.getAngle());
                        turretValues.addNumber("TUTgt", () -> turret.targetAngle);
                        turretValues.addNumber("Pct", () -> turret.getOut());
                        turretValues.addNumber("Amps", () -> turret.getAmps());
                        turretValues.addNumber("Speed", () -> turret.getSpeed());
                        turretValues.addNumber("Vision Offset", () -> turret.targetHorizontalOffset);
                        turretValues.addNumber("AdjTarget", () -> turret.adjustedCameraError);
                        turretValues.addNumber("Vision Error", () -> limelight.getdegRotationToTarget());
                        turretValues.addNumber("DriverOffset", () -> turret.driverHorizontalOffsetDegrees);
                        turretValues.addNumber("LockPosnErr", () -> turret.getLockPositionError());
                        turretValues.addBoolean("MinHWLim", () -> turret.onMinusHardwareLimit());
                        turretValues.addBoolean("PlusHWLim", () -> turret.onPlusHardwareLimit());

                        ShuffleboardLayout turretValues3 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2,
                                                        2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretValues3.addNumber("IAccum", () -> turret.getIaccum());

                        turretValues3.addNumber("LockOutput", () -> turret.lockPIDOut);
                        turretValues3.addNumber("LockError", () -> turret.m_turretLockController.getPositionError());
                        turretValues3.addBoolean("LockController", () -> turret.validTargetSeen);

                        ShuffleboardLayout turretValues2 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 2).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels
                        turretValues2.addBoolean("Connected (8)", () -> turret.turretMotorConnected);

                        turretValues2.addBoolean("PlusLimit", () -> turret.onPlusSoftwareLimit());

                        turretValues2.addBoolean("MinusLimit", () -> turret.onMinusSoftwareLimit());

                        turretValues2.addBoolean("SWLimitEn", () -> turret.getSoftwareLimitsEnabled());

                        turretValues2.addBoolean("InPosition", () -> turret.atTargetAngle());

                        turretValues2.addBoolean("BrakeMode", () -> turret.isBrake());
                        turretValues2.addBoolean("TargetHorOK",
                                        () -> limelight.getHorOnTarget(turret.turretVisionTolerance));

                        turretValues2.addBoolean("OKTune", () -> (turret.tuneOnv &&
                                        turret.lastTuneOnv));

                        turretValues2.addBoolean("LockAtTarget", () -> turret.getLockAtTarget());

                        ShuffleboardLayout turretGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        turretGains.addNumber("FF", () -> turret.ffset);
                        turretGains.addNumber("P", () -> turret.pset);
                        turretGains.addNumber("I", () -> turret.iset);
                        turretGains.addNumber("D", () -> turret.dset);
                        turretGains.addNumber("IZ", () -> turret.izset);
                        turretGains.addNumber("MaxAcc", () -> turret.maxAccset);
                        turretGains.addNumber("MaxV", () -> turret.maxVelset);

                        ShuffleboardLayout turretLockGains = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("LockGains", BuiltInLayouts.kList)
                                        .withPosition(7, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        turretLockGains.addNumber("LP", () -> turret.lpset);
                        turretLockGains.addNumber("LI", () -> turret.liset);
                        turretLockGains.addNumber("LD", () -> turret.ldset);
                        turretGains.addNumber("LIZ", () -> turret.lizset);

                }

                if (showTilt && !isMatch) {

                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Position To 10", new PositionTilt(tilt, 10));

                        tiltCommands.add("Position To 14", new PositionTilt(tilt, 13));
                        tiltCommands.add("Position To 5", new PositionTilt(tilt, 5));
                        tiltCommands.add("PositionToSwitch", new TiltMoveToReverseLimit(tilt));
                        tiltCommands.add("StopTilt", new StopTilt(tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(tilt));
                        tiltCommands.add("Cmd", tilt);
                        tiltCommands.addNumber("Faults", () -> tilt.faultSeen);

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TIAngle", () -> tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> tilt.targetAngle);
                        tiltValues.addNumber("PCT", () -> tilt.getOut());
                        tiltValues.addNumber("Amps", () -> tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> tilt.getSpeed());
                        // tiltValues.addNumber("Vision Offset", () -> tilt.targetVerticalOffset);
                        tiltValues.addNumber("AdjTarget", () -> tilt.adjustedVerticalError);
                        // tiltValues.addNumber("Vision Error", () ->
                        // limelight.getdegVerticalToTarget());

                        tiltValues.addNumber("DriverOffset", () -> tilt.driverVerticalOffsetDegrees);

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 0).withSize(3, 2)
                                        .withProperties(Map.of("Label position", "TOP"));

                        tiltValues2.addBoolean("InPosition", () -> tilt.atTargetAngle());

                        tiltValues2.addBoolean("OnBottomLS", () -> tilt.m_reverseLimit.isPressed());

                        tiltValues2.addBoolean("PosResetDone", () -> tilt.positionResetDone);
                        tiltValues2.addBoolean("BrakeMode", () -> tilt.isBrake());
                        tiltValues2.addBoolean("OKTune", () -> (tilt.tuneOn && tilt.lastTuneOn));
                        tiltValues2.addBoolean("Connected (9)", () -> tilt.tiltMotorConnected);
                        tiltValues2.addBoolean("+SWLimit", () -> tilt.onPlusSoftwareLimit());
                        tiltValues2.addBoolean("-SWLimit", () -> tilt.onMinusSoftwareLimit());
                        tiltValues2.addBoolean("SWLimitEn", () -> tilt.getSoftwareLimitsEnabled());

                        ShuffleboardLayout tiltGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 3)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        tiltGains.addNumber("FF", () -> tilt.ffset);
                        tiltGains.addNumber("P", () -> tilt.pset);
                        tiltGains.addNumber("I", () -> tilt.iset);
                        tiltGains.addNumber("D", () -> tilt.dset);
                        tiltGains.addNumber("IZ", () -> tilt.izset);
                        tiltGains.addNumber("MaxAcc", () -> tilt.maxAccset);
                        tiltGains.addNumber("MaxV", () -> tilt.maxVelset);

                }

                if (showShooter && !isMatch) {

                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("MAXRPM 3500", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        shooterCommands.add("Stop Shoot", new StopShoot(shooter, transport));

                        shooterCommands.add("Shoot",
                                        new ShootCargo(shooter, 0, transport, intake));
                        shooterCommands.add("ClearFaults", new ClearShFaults(shooter));
                        shooterCommands.add("Cmd", shooter);

                        shooterCommands.add("RunShooterFromSlider",
                                        new RunShooter(shooter, shooter.shooterSpeed.getDouble(500)));

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues.addNumber("RPM", () -> shooter.getRPM());
                        shooterValues.addNumber("Left PCT", () -> shooter.getLeftPctOut());
                        shooterValues.addNumber("LeftAmps", () -> shooter.getLeftAmps());

                        shooterValues.addNumber("RightAmps", () -> shooter.getRightAmps());
                        shooterValues.addNumber("SpeedCommand RPM", () -> shooter.requiredRPM);
                        shooterValues.addNumber("LeftFaults", () -> shooter.getLeftFaults());
                        shooterValues.addNumber("RightFaults", () -> shooter.getRightFaults());

                        ShuffleboardLayout shooterValues1 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("ShooterValues1", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                        shooterValues1.addNumber("TargetDistance", () -> shooter.calculatedCameraDistance);

                        shooterValues1.addBoolean("ShooterAtSpeed", () -> shooter.atSpeed());
                        shooterValues1.addBoolean("TopRollerAtSpeed", () -> shooter.getTopRollerAtSpeed());

                        shooterValues1.addBoolean("TuneOn", () -> (shooter.tuneOn && shooter.lastTuneOn));
                        shooterValues1.addBoolean("BothConnected(6,7)", () -> shooter.allConnected);
                        shooterValues1.addBoolean("DriverOKShoot", () -> shooter.driverOKShoot);
                        shooterValues1.addBoolean("ShootOne", () -> shooter.shootOne);
                        shooterValues1.addBoolean("Use Slider", () -> shooter.useSpeedSlider);

                        ShuffleboardLayout shooterValues2 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("Gains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues2.addNumber("FF", () -> shooter.ffset);
                        shooterValues2.addNumber("P", () -> shooter.pset);
                        shooterValues2.addNumber("I", () -> shooter.iset);
                        shooterValues2.addNumber("D", () -> shooter.dset);
                        shooterValues2.addNumber("IZ", () -> shooter.izset);

                }

                if (showTransport && !isMatch) {

                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 5).withProperties(Map.of("Label position", "LEFT")); // labels

                        transportValues.add("StartLowerRoller", new RunLowerRoller(transport, 500));
                        transportValues.add("StopLowerRoller", new StopLowerRoller(transport));

                        transportValues.addNumber("LowerRollerRPM", () -> transport.getLowerRPM());
                        transportValues.addNumber("LowerRollerAmps", () -> transport.getLowerRollerMotorAmps());
                        transportValues.addNumber("LowerRollerOut", () -> transport.getLowerRoller());
                        transportValues.add("Cmd", transport);
                        transportValues.addBoolean("BlueCargo", () -> transport.getCargoIsBlue());
                        transportValues.addBoolean("RedCargo", () -> transport.getCargoIsRed());
                        transportValues.addBoolean("CargoAtShoot", () -> transport.getCargoAtShoot());

                        ShuffleboardLayout transportValues1 = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportStates", BuiltInLayouts.kGrid).withPosition(4, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // label

                        transportValues1.addBoolean("LowerRollerAtSpeed", () -> transport.getLowerRollerAtSpeed());

                        transportValues1.addBoolean("LowerRollerConnected (14)",
                                        () -> transport.lowerRollerMotorConnected);

                }

                if (showClimber && !isMatch) {
                        ShuffleboardLayout climberInfo = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("Climber", BuiltInLayouts.kList).withPosition(7, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        climberInfo.addBoolean("ClimbArmUp", () -> climber.getArmRaised());
                        climberInfo.addBoolean("ClimbArmUDown", () -> climber.getArmLowered());
                        climberInfo.addBoolean("Locked", () -> climber.getRatchetLocked());
                        climberInfo.addBoolean("Unlocked", () -> climber.getRatchetUnlocked());

                }

                if (showRobot && !isMatch) {

                        ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        robotCommands.add("Reset Enc", new ResetEncoders(drive));
                        robotCommands.add("Reset Gyro", new ResetGyro(drive));

                        robotCommands.add("ClearFaults", new ClearRobFaults(drive));
                        robotCommands.add("Stop Robot", new StopRobot(drive));
                        robotCommands.add("To -4(2)", new PositionMove(drive, -4, 2));
                        robotCommands.add("To -4(3)", new PositionMove(drive, -4, 3));
                        robotCommands.add("To -4(1)", new PositionMove(drive, -4, 1));
                        robotCommands.add("To 0(1)", new PositionMove(drive, 0, 1));
                        robotCommands.add("To 0", new PositionMove(drive,  0, .5));
                        robotCommands.add("Cmd", drive);

                        ShuffleboardLayout robotValues = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("RobotValues", BuiltInLayouts.kList).withPosition(2, 0)
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

                        ShuffleboardLayout robotValues2 = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(5, 0)
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
                                        .getLayout("Gains", BuiltInLayouts.kGrid).withPosition(6, 0)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        robotGains.addNumber("LFF", () -> drive.ffset);
                        robotGains.addNumber("LP", () -> drive.pset);
                        robotGains.addNumber("RFF", () -> drive.rffset);
                        robotGains.addNumber("RP", () -> drive.rpset);

                }

                if (showSubsystems && !isMatch) {

                        ShuffleboardLayout subSystems = Shuffleboard.getTab("Can+Sols")
                                        .getLayout("All", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(3, 7).withProperties(Map.of("Label position", "LEFT")); //

                        subSystems.add("Drive", drive);
                        subSystems.add("Shooter",
                                        shooter);
                        subSystems.add("Turret", turret);
                        subSystems.add("Tilt", tilt);
                        subSystems.add("Transport", transport);
                        subSystems.add("Intakes", intake);

                        ShuffleboardLayout scheduler = Shuffleboard.getTab("Can+Sols")
                                        .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(3, 0)
                                        .withSize(7, 2).withProperties(Map.of("Label position", "TOP")); //

                        scheduler.add("Scheduler", CommandScheduler.getInstance());

                        ShuffleboardLayout canBus = Shuffleboard.getTab("Can+Sols")
                                        .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(3, 2)
                                        .withSize(4, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        canBus.addBoolean("TurretConnected (8)", () -> turret.turretMotorConnected);
                        canBus.addBoolean("TiltConnected (9)", () -> tilt.tiltMotorConnected);
                        // canBus.addBoolean("LeftShooterConnected (6)
                        // canBus.addBoolean("TopRollerConnected (12)", () ->
                        // transport.topRollerMotorConnected);
                        canBus.addBoolean("LowerRollerConnected(14)", () -> transport.lowerRollerMotorConnected);
                        canBus.addBoolean("LDR1Connected  (2)", () -> drive.leftLeadConnected);

                        // (4)",()->robotDrive.rightLeadConnected)
                        canBus.addBoolean("RDr2Connected (5)", () -> drive.rightFollowerConnected);
                        canBus.addBoolean("FrontIntakeConnected (10)", () -> intake.frontIntakeMotorConnected);
                        canBus.addBoolean("RearIntakeConnected (10)", () -> intake.rearIntakeMotorConnected);
                }
        }

}