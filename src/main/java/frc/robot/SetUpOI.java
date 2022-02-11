// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CargoTransport.HoldCargo;
import frc.robot.commands.CargoTransport.ReleaseCargo;
import frc.robot.commands.CargoTransport.RunRollers;
import frc.robot.commands.CargoTransport.StopRollers;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntakeMotor;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopIntakeMotors;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.EndDriveLog;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.PickupMoveVelocity;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.Shooter.ChooseShooterSpeedSource;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.EndShootLog;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
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
        private final RevTiltSubsystem m_tilt;
        private final RevTurretSubsystem m_turret;
        private final RevDrivetrain m_robotDrive;
        private final RevShooterSubsystem m_shooter;
        private final CargoTransportSubsystem m_transport;
        private final Compressor m_compressor;
        private final LimeLight m_limelight;
        private final IntakesSubsystem m_intake;
        private final ClimberSubsystem m_climber;
        private final AngleSolver m_as;

        private boolean m_showTurret = true;
        private boolean m_showTilt = true;
        private boolean m_showShooter = true;
        private boolean m_showRobot = false;
        private boolean m_showTransport = false;
        private boolean m_showClimber = false;
        private boolean m_showSubsystems = false;
        private boolean m_showIntake = false;

        private HttpCamera LLFeed;
        private UsbCamera intakeFeed;
        public double timeToStart;

        public SetUpOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, IntakesSubsystem intake,
                        ClimberSubsystem climber,
                        FondyFireTrajectory traj, AngleSolver as, boolean liveMatch) {
                m_turret = turret;
                m_tilt = tilt;
                m_robotDrive = drive;
                m_transport = transport;
                m_compressor = compressor;
                m_shooter = shooter;
                m_limelight = limelight;
                m_intake = intake;
                m_climber = climber;
                m_as = as;

                if (m_showIntake) {

                        ShuffleboardLayout intakeSelect = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeSelect", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "TOP"));

                        intakeSelect.add("Select Front", new SetFrontIntakeActive(m_intake));
                        intakeSelect.add("Select Rear", new SetRearIntakeActive(m_intake));
                        intakeSelect.addBoolean("RearActive", () -> !m_intake.useFrontIntake);
                        intakeSelect.addBoolean("FrontActive", () -> m_intake.useFrontIntake);

                        ShuffleboardLayout intakeActions = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeActions", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP"));

                        intakeActions.add("ArmRaise", new ActiveIntakeArmRaise(m_intake));
                        intakeActions.add("ArmLower", new ActiveIntakeArmLower(m_intake));
                        intakeActions.add("Run Motor", new RunActiveIntakeMotor(intake, .75));
                        intakeActions.add("Stop Motor", new StopIntakeMotors(intake));

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(2, 2)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP"));
                        intakeValues.addBoolean("Arm Up", () -> m_intake.getActiveArmRaised());
                        intakeValues.addBoolean("Arm Down", () -> m_intake.getActiveArmLowered());
                        intakeValues.addNumber("Motor Amps", () -> m_intake.getActiveMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> m_intake.getActiveMotor());

                        // Shuffleboard.getTab("Intake").add("Intake", intakeFeed)
                        // .withWidget(BuiltInWidgets.kCameraStream)
                        // .withPosition(2, 0).withSize(6, 4)
                        // .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//
                }

                if (m_showTurret) {

                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretCommands.add("Reset to 0", new ResetTurretAngle(m_turret));
                        turretCommands.add("Position To 0", new PositionTurret(m_turret, 0));//
                        // degrees
                        turretCommands.add("Position To -10", new PositionTurret(m_turret, -10));//

                        turretCommands.add("Position To 10", new PositionTurret(m_turret, 10));
                        turretCommands.add("PosToVis +10", new PositionTurretToVision(m_turret,
                                        m_limelight, 10));
                        turretCommands.add("PosToVis-10", new PositionTurretToVision(m_turret,
                                        m_limelight, -10));

                        turretCommands.add("StopTurret", new StopTurret(m_turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(m_turret));
                        turretCommands.add("Cmd", m_turret);
                        turretCommands.addNumber("Faults", () -> m_turret.getFaults());
                        turretCommands.addString("To Jog", () -> "SetupXBox Btn A left X");
                        turretCommands.addString("OvrRideSoftLim", () -> "Setup RightBmpr");

                        ShuffleboardLayout turretValues = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("TurretValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels

                        turretValues.addNumber("TUAngle", () -> m_turret.getAngle());
                        turretValues.addNumber("TUTgt", () -> m_turret.targetAngle);
                        turretValues.addNumber("Pct", () -> m_turret.getOut());
                        turretValues.addNumber("Amps", () -> m_turret.getAmps());
                        turretValues.addNumber("Speed", () -> m_turret.getSpeed());
                        turretValues.addNumber("Vision Offset", () -> m_turret.targetHorizontalOffset);
                        turretValues.addNumber("AdjTarget", () -> m_turret.adjustedCameraError);
                        turretValues.addNumber("Vision Error", () -> m_limelight.getdegRotationToTarget());
                        turretValues.addNumber("DriverOffset", () -> m_turret.driverHorizontalOffsetDegrees);
                        turretValues.addNumber("LockPosnErr", () -> m_turret.getLockPositionError());

                        ShuffleboardLayout turretValues3 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2,
                                                        2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretValues3.addNumber("IAccum", () -> m_turret.getIaccum());

                        turretValues3.addNumber("LockOutput", () -> m_turret.lockPIDOut);
                        turretValues3.addNumber("LockError", () -> m_turret.m_turretLockController.getPositionError());
                        turretValues3.addBoolean("LockController", () -> m_turret.validTargetSeen);

                        ShuffleboardLayout turretValues2 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(2, 3).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels
                        turretValues2.addBoolean("Connected (8)", () -> m_turret.turretMotorConnected);

                        turretValues2.addBoolean("PlusLimit", () -> m_turret.onPlusSoftwareLimit());

                        turretValues2.addBoolean("MinusLimit", () -> m_turret.onMinusSoftwareLimit());

                        turretValues2.addBoolean("SWLimitEn", () -> m_turret.getSoftwareLimitsEnabled());

                        turretValues2.addBoolean("InPosition", () -> m_turret.atTargetAngle());

                        turretValues2.addBoolean("BrakeMode", () -> m_turret.isBrake());
                        turretValues2.addBoolean("TargetHorOK",
                                        () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));

                        turretValues2.addBoolean("OKTune", () -> (m_turret.tuneOnv &&
                                        m_turret.lastTuneOnv));

                        turretValues2.addBoolean("LockAtTarget", () -> m_turret.getLockAtTarget());

                        ShuffleboardLayout turretGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        turretGains.addNumber("FF", () -> m_turret.ffset);
                        turretGains.addNumber("P", () -> m_turret.pset);
                        turretGains.addNumber("I", () -> m_turret.iset);
                        turretGains.addNumber("D", () -> m_turret.dset);
                        turretGains.addNumber("IZ", () -> m_turret.izset);
                        turretGains.addNumber("MaxAcc", () -> m_turret.maxAccset);
                        turretGains.addNumber("MaxV", () -> m_turret.maxVelset);

                        ShuffleboardLayout turretVGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("VelGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        turretVGains.addNumber("FF", () -> m_turret.ffsetv);
                        turretVGains.addNumber("P", () -> m_turret.psetv);
                        turretVGains.addNumber("I", () -> m_turret.isetv);
                        turretVGains.addNumber("D", () -> m_turret.dsetv);
                        turretVGains.addNumber("IZ", () -> m_turret.izsetv);

                        ShuffleboardLayout turretLockGains = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("LockGains", BuiltInLayouts.kList)
                                        .withPosition(6, 1).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        turretLockGains.addNumber("LP", () -> m_turret.lpset);
                        turretLockGains.addNumber("LI", () -> m_turret.liset);
                        turretLockGains.addNumber("LD", () -> m_turret.ldset);
                        turretGains.addNumber("LIZ", () -> m_turret.lizset);

                }

                if (m_showTilt) {

                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Position To 25", new PositionTilt(m_tilt, 25));

                        tiltCommands.add("Position To 15", new PositionTilt(m_tilt, 15));
                        tiltCommands.add("Position To 5", new PositionTilt(m_tilt, 5));
                        tiltCommands.add("PosToVis + ", new PositionTiltToVision(m_tilt, m_limelight, 10));

                        tiltCommands.add("PosToVis -",
                                        new PositionTiltToVision(m_tilt, m_limelight, m_tilt.tiltMinAngle));
                        tiltCommands.add("PositionToSwitch", new TiltMoveToReverseLimit(m_tilt));
                        tiltCommands.add("StopTilt", new StopTilt(m_tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(m_tilt));
                        tiltCommands.add("Cmd", m_tilt);
                        tiltCommands.addNumber("Faults", () -> m_tilt.faultSeen);

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TICameraAngle", () -> m_tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> m_tilt.targetAngle);
                        tiltValues.addNumber("PCT", () -> m_tilt.getOut());
                        tiltValues.addNumber("Amps", () -> m_tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> m_tilt.getSpeed());
                        tiltValues.addNumber("Vision Offset", () -> m_tilt.targetVerticalOffset);
                        tiltValues.addNumber("AdjTarget", () -> m_tilt.adjustedVerticalError);
                        tiltValues.addNumber("Vision Error", () -> m_limelight.getdegVerticalToTarget());
                        tiltValues.addNumber("MotorDeg", () -> m_tilt.getMotorDegrees());
                        tiltValues.addNumber("MotorTarget", () -> m_tilt.motorEndpointDegrees);
                        tiltValues.addNumber("DriverOffset", () -> m_tilt.driverVerticalOffsetDegrees);
                        tiltValues.addNumber("LockError", () -> m_tilt.tiltLockController.getPositionError());

                        ShuffleboardLayout tiltValues3 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        tiltValues3.addNumber("IAccum", () -> m_tilt.getIaccum());

                        tiltValues3.addNumber("LockOutput", () -> m_tilt.lockPIDOut);
                        tiltValues3.addNumber("LockError", () -> m_tilt.tiltLockController.getPositionError());
                        tiltValues3.addBoolean(("LockController"), () -> m_tilt.validTargetSeen);
                        tiltValues3.addBoolean("LockOnTarget", () -> m_tilt.getLockAtTarget());
                        tiltValues3.addBoolean("ValTgt", () -> m_tilt.validTargetSeen);

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 2).withSize(3, 2)
                                        .withProperties(Map.of("Label position", "TOP"));

                        tiltValues2.addBoolean("InPosition", () -> m_tilt.atTargetAngle());

                        tiltValues2.addBoolean("OnBottomLS", () -> m_tilt.m_reverseLimit.isPressed());

                        tiltValues2.addBoolean("PosResetDone", () -> m_tilt.positionResetDone);
                        tiltValues2.addBoolean("BrakeMode", () -> m_tilt.isBrake());
                        tiltValues2.addBoolean("OKTune", () -> (m_tilt.tuneOn && m_tilt.lastTuneOn));
                        tiltValues2.addBoolean("Connected (9)", () -> m_tilt.tiltMotorConnected);
                        tiltValues2.addBoolean("+SWLimit", () -> m_tilt.onPlusSoftwareLimit());
                        tiltValues2.addBoolean("-SWLimit", () -> m_tilt.onMinusSoftwareLimit());
                        tiltValues2.addBoolean("SWLimitEn", () -> m_tilt.getSoftwareLimitsEnabled());
                        tiltValues2.addBoolean("TargetVertOK",
                                        () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));

                        ShuffleboardLayout tiltGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        tiltGains.addNumber("FF", () -> m_tilt.ffset);
                        tiltGains.addNumber("P", () -> m_tilt.pset);
                        tiltGains.addNumber("I", () -> m_tilt.iset);
                        tiltGains.addNumber("D", () -> m_tilt.dset);
                        tiltGains.addNumber("IZ", () -> m_tilt.izset);
                        tiltGains.addNumber("MaxAcc", () -> m_tilt.maxAccset);
                        tiltGains.addNumber("MaxV", () -> m_tilt.maxVelset);
                        ShuffleboardLayout tiltVGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("VelGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        tiltVGains.addNumber("FF", () -> m_tilt.ffsetv);
                        tiltVGains.addNumber("P", () -> m_tilt.psetv);
                        tiltVGains.addNumber("I", () -> m_tilt.isetv);
                        tiltVGains.addNumber("D", () -> m_tilt.dsetv);
                        tiltVGains.addNumber("IZ", () -> m_tilt.izsetv);

                        ShuffleboardLayout tiltLockGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("LockGains", BuiltInLayouts.kList).withPosition(6, 1).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        tiltLockGains.addNumber("LP", () -> m_tilt.lpset);
                        tiltLockGains.addNumber("LI", () -> m_tilt.liset);
                        tiltLockGains.addNumber("LD", () -> m_tilt.ldset);
                        tiltLockGains.addNumber("LIZ", () -> m_tilt.lizset);

                }

                if (m_showShooter) {

                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("MAXMPS 50", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        shooterCommands.add("Stop Shoot", new StopShoot(m_shooter, m_transport));
                        shooterCommands.add("Shoot",
                                        new ShootCargo(m_shooter, m_tilt, m_turret, m_limelight, m_transport, drive,
                                                        m_compressor, 0));
                        shooterCommands.add("ClearFaults", new ClearShFaults(m_shooter));
                        shooterCommands.add("Cmd", m_shooter);
                        shooterCommands.add("EndLog", new EndShootLog(shooter));
                        
                        shooterCommands.add("RunAllShooters", new RunShooter(shooter));
                        shooterCommands.add("UseSpeedSlider", new ChooseShooterSpeedSource(shooter, tilt, turret, 3));

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues.addNumber("LeftMPS", () -> m_shooter.getMPS());
                        shooterValues.addNumber("Left PCT", () -> m_shooter.getLeftPctOut());
                        shooterValues.addNumber("LeftAmps", () -> m_shooter.getLeftAmps());
                        shooterValues.addNumber("RightRPM", () -> m_shooter.getRightRPM());
                        shooterValues.addNumber("RightAmps", () -> m_shooter.getRightAmps());
                        shooterValues.addNumber("SpeedCommand MPS", () -> m_shooter.requiredMps);
                        shooterValues.addNumber("LeftFaults", () -> m_shooter.getLeftFaults());
                        shooterValues.addNumber("RightFaults", () -> m_shooter.getRightFaults());
                        shooterValues.addBoolean("CameraHasSpeed", () -> m_shooter.useCameraSpeed);

                        ShuffleboardLayout shooterValues1 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("ShooterValues1", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                        shooterValues1.addNumber("VertOffset", () -> m_tilt.targetVerticalOffset);
                        shooterValues1.addNumber("HorOffset", () -> m_turret.targetHorizontalOffset);
                        shooterValues1.addNumber("DriverVOffset", () -> m_tilt.driverVerticalOffsetDegrees);
                        shooterValues1.addNumber("DriverHOffset", () -> m_turret.driverHorizontalOffsetDegrees);

                        shooterValues1.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        shooterValues1.addNumber("CameraAngle", () -> m_tilt.getCameraAngle());

                        shooterValues1.addBoolean("AtSpeed", () -> m_shooter.atSpeed());
                        shooterValues1.addBoolean("TuneOn", () -> (m_shooter.tuneOn && m_shooter.lastTuneOn));
                        shooterValues1.addBoolean("BothConnected(6,7)", () -> m_shooter.allConnected);
                        shooterValues1.addBoolean("DriverOKShoot", () -> m_shooter.driverOKShoot);
                        shooterValues1.addBoolean("ShootOne", () -> m_shooter.shootOne);
                        shooterValues1.addBoolean("UsingSliders", () -> m_shooter.useSetupSlider);

                        ShuffleboardLayout shooterValues2 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("Gains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues2.addNumber("FF", () -> m_shooter.ffset);
                        shooterValues2.addNumber("P", () -> m_shooter.pset);
                        shooterValues2.addNumber("I", () -> m_shooter.iset);
                        shooterValues2.addNumber("D", () -> m_shooter.dset);
                        shooterValues2.addNumber("IZ", () -> m_shooter.izset);

                        m_shooter.shooterSpeed = Shuffleboard.getTab("SetupShooter").add("ShooterSpeed", 3)
                                        .withWidget("NumberSlider")
                                        .withPosition(0, 3).withSize(4, 1).withProperties(Map.of("Min", 15, "Max", 50))
                                        .getEntry();

                }

                if (m_showTransport) {

                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        transportValues.add("StartRollers", new RunRollers(transport));
                        transportValues.add("StopRollers", new StopRollers(transport));

                        transportValues.addNumber("FrontRollerAmps", () -> m_transport.getFrontRollerMotorAmps());
                        transportValues.addNumber("RearRollerAmps", () -> m_transport.getRearRollerMotorAmps());
                        transportValues.addNumber("FrontRollerOut", () -> m_transport.getFrontRoller());
                        transportValues.addNumber("RearRollerOut", () -> m_transport.getRearRoller());
                        transportValues.add("Cmd", m_transport);

                        ShuffleboardLayout transportCargoArm = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportCargoArm", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        transportCargoArm.add("Hold Cargo", new HoldCargo(transport));
                        transportCargoArm.add("ReleaseOneCargo", new ReleaseCargo(transport));

                        transportCargoArm.add("Release Cargo", new ReleaseCargo(transport));

                        ShuffleboardLayout transportValues1 = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportStates", BuiltInLayouts.kGrid).withPosition(4, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // label

                        transportValues1.addBoolean("RollersAtSpeed", () -> m_transport.rollersAtSpeed);

                        transportValues1.addBoolean("RearRollerConnected (12)",
                                        () -> m_transport.rearRollerMotorConnected);
                        transportValues1.addBoolean("FrontRollerConnected (14)",
                                        () -> m_transport.frontRollerMotorConnected);

                }

                if (m_showClimber) {
                        ShuffleboardLayout climberInfo = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("Climber", BuiltInLayouts.kList).withPosition(7, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        climberInfo.addBoolean("ClimbArmUp", () -> m_climber.getArmRaised());
                        climberInfo.addBoolean("ClimbArmUDown", () -> m_climber.getArmLowered());
                        climberInfo.addBoolean("Locked", () -> m_climber.getRatchetLocked());
                        climberInfo.addBoolean("Unlocked", () -> m_climber.getRatchetUnlocked());

                }

                if (m_showRobot) {

                        ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        robotCommands.add("Reset Enc", new ResetEncoders(m_robotDrive));
                        robotCommands.add("Reset Gyro", new ResetGyro(m_robotDrive));

                        robotCommands.add("ClearFaults", new ClearRobFaults(m_robotDrive));
                        robotCommands.add("Stop Robot", new StopRobot(m_robotDrive));
                        robotCommands.add("To -4(2)", new PickupMoveVelocity(m_robotDrive, -4, 2));
                        robotCommands.add("To -4(3)", new PickupMoveVelocity(m_robotDrive, -4, 3));
                        robotCommands.add("To -4(1)", new PickupMoveVelocity(m_robotDrive, -4, 1));
                        robotCommands.add("To 0(1)", new PickupMoveVelocity(m_robotDrive, 0, 1));
                        robotCommands.add("To 0", new PickupMove(m_robotDrive, 0, .5));
                        robotCommands.add("Cmd", m_robotDrive);

                        robotCommands.add("EndLog", new EndDriveLog(m_robotDrive));

                        ShuffleboardLayout robotValues = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("RobotValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        robotValues.addNumber("LeftMeters", () -> m_robotDrive.getLeftDistance());
                        robotValues.addNumber("RightMeters", () -> m_robotDrive.getRightDistance());
                        robotValues.addNumber("LeftVelMPS", () -> m_robotDrive.getLeftRate());
                        robotValues.addNumber("RightVelMPS", () -> m_robotDrive.getRightRate());
                        robotValues.addNumber("LeftOut", () -> m_robotDrive.getLeftOut());
                        robotValues.addNumber("RightOut", () -> m_robotDrive.getRightOut());
                        robotValues.addNumber("LeftAmps", () -> m_robotDrive.getLeftAmps());
                        robotValues.addNumber("RightAmps", () -> m_robotDrive.getRightAmps());
                        robotValues.addNumber("Gyro Yaw", () -> m_robotDrive.getYaw());
                        robotValues.addNumber("Faults", () -> m_robotDrive.getFaults());
                        robotValues.addNumber("Target", () -> m_robotDrive.leftTargetPosition);

                        ShuffleboardLayout robotValues2 = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(5, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels

                        robotValues2.addBoolean("TuneOn", () -> (m_robotDrive.tuneOn && m_robotDrive.lastTuneOn));
                        robotValues2.addBoolean("Left1Connected  (2)", () -> m_robotDrive.leftLeadConnected);
                        robotValues2.addBoolean("Left2Connected (3)", () -> m_robotDrive.leftFollowerConnected);
                        robotValues2.addBoolean("Right1Connected (4)", () -> m_robotDrive.rightLeadConnected);
                        robotValues2.addBoolean("Right2Connected  (5)", () -> m_robotDrive.rightFollowerConnected);
                        robotValues2.addBoolean("LInPosition", () -> m_robotDrive.getInPositionLeft());
                        robotValues2.addBoolean("RInPosition", () -> m_robotDrive.getInPositionRight());
                        robotValues2.addBoolean("LFoll", () -> m_robotDrive.getLeftFollower());
                        robotValues2.addBoolean("RFoll", () -> m_robotDrive.getRightFollower());

                        ShuffleboardLayout robotGains = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Gains", BuiltInLayouts.kGrid).withPosition(6, 0)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        robotGains.addNumber("LFF", () -> m_robotDrive.ffset);
                        robotGains.addNumber("LP", () -> m_robotDrive.pset);
                        robotGains.addNumber("RFF", () -> m_robotDrive.rffset);
                        robotGains.addNumber("RP", () -> m_robotDrive.rpset);

                }

                if (m_showSubsystems) {

                        ShuffleboardLayout subSystems = Shuffleboard.getTab("Can+Sols")
                                        .getLayout("All", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(3, 7).withProperties(Map.of("Label position", "LEFT")); //

                        subSystems.add("Drive", m_robotDrive);
                        subSystems.add("Shooter",
                                        m_shooter);
                        subSystems.add("Turret", m_turret);
                        subSystems.add("Tilt", m_tilt);
                        subSystems.add("Transport", m_transport);
                        subSystems.add("Intakes", m_intake);

                        ShuffleboardLayout scheduler = Shuffleboard.getTab("Can+Sols")
                                        .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(3, 0)
                                        .withSize(7, 2).withProperties(Map.of("Label position", "TOP")); //

                        scheduler.add("Scheduler", CommandScheduler.getInstance());

                        ShuffleboardLayout canBus = Shuffleboard.getTab("Pre-Round")
                                        .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(3, 2)
                                        .withSize(4, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        canBus.addBoolean("TurretConnected (8)", () -> m_turret.turretMotorConnected);
                        canBus.addBoolean("TiltConnected (9)", () -> m_tilt.tiltMotorConnected);
                        // canBus.addBoolean("LeftShooterConnected (6)
                        canBus.addBoolean("RearRollerConnected (12)", () -> m_transport.rearRollerMotorConnected);
                        canBus.addBoolean("FrontRollerConnected(14)", () -> m_transport.frontRollerMotorConnected);
                        canBus.addBoolean("LDR1Connected  (2)", () -> m_robotDrive.leftLeadConnected);

                        // (4)",()->m_robotDrive.rightLeadConnected)
                        canBus.addBoolean("RDr2Connected (5)", () -> m_robotDrive.rightFollowerConnected);
                        canBus.addBoolean("FrontIntakeConnected (10)", () -> m_intake.frontIntakeMotorConnected);
                        canBus.addBoolean("RearIntakeConnected (10)", () -> m_intake.rearIntakeMotorConnected);
                }
        }

        public void checkCANDevices() {
                m_turret.checkCAN();
                m_tilt.checkCAN();
                m_intake.checkFrontCAN();
                m_intake.checkRearCAN();
                m_shooter.checkCAN();
                m_robotDrive.checkCAN();
                m_transport.checkCAN();

        }

        public double[] getPDPInfo() {
                double temp[] = { 0, 0, 0, 0, 0 };
                temp[0] = m_shooter.getBatteryVoltage();
                temp[1] = m_shooter.getTemperature();
                temp[2] = m_shooter.getTotalEnergy() / 3600;
                temp[3] = m_shooter.getTotalPower();
                return temp;

        }

        public void checkLimits() {
                if (m_tilt.onMinusSoftwareLimit() || m_tilt.onPlusSoftwareLimit() || m_tilt.onMinusHardwarLimit()
                                || m_turret.onPlusSoftwareLimit() || m_turret.onMinusSoftwareLimit()
                                || DriverStation.isDisabled())
                        m_limelight.useVision = false;

        }

}