// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class RightPUShootThird extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public RightPUShootThird(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll, Compressor comp, double[] data) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;
                double positionRate = drive.positionRate;

                double drivePickupPosition = data[0];
                double shootPosition = data[1];
                double upperTiltAngle = data[2];
                double upperTurretAngle = data[3];
                double upperRPM = data[4];
                // remaining data used in shoot routine
                double pickUpAngle = -40;
                double shootAngleDegrees = 110;

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, true),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelCommandGroup(

                                                new TurnToAngle(drive, pickUpAngle),

                                                new ResetEncoders(drive),

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new PositionTilt(tilt, upperTiltAngle),

                                                new WaitCommand(2)

                                                                .deadlineWith(new RunActiveIntake(intake, transport))),

                                new SequentialCommandGroup(

                                                new ParallelCommandGroup(
                                                                new SetUpLimelightForTarget(ll,
                                                                                PipelinesConstants.noZoom960720, false),
                                                                new SetFrontIntakeActive(intake, false),
                                                                new ResetEncoders(drive),
                                                                new ResetGyro(drive),

                                                                new TurnToAngle(drive, shootAngleDegrees),
                                                                new ResetEncoders(drive),
                                                                new ResetGyro(drive)),

                                                new ParallelCommandGroup(

                                                                new SetPresetRPM(shooter, upperRPM),
                                                                new LimelightSetPipeline(ll,
                                                                                PipelinesConstants.noZoom960720),
                                                                new UseVision(ll, true),

                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(.2),
                                                                                new AltShootCargo(
                                                                                                shooter,
                                                                                                transport,
                                                                                                intake,
                                                                                                ll),

                                                                                new WaitCommand(.1))),

                                                new PositionTurret(turret, 0)));

        }
}