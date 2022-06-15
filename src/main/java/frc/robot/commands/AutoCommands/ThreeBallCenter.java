// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngleProfiled;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class ThreeBallCenter extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public ThreeBallCenter(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll, Compressor comp) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;
                double positionRate = drive.positionRate;

                double drivePickupPosition = Units.inchesToMeters(-51.);
                double upperTiltAngle = 13.;
                double upperTurretAngle = -17.;
                double upperRPM = 3400.;

                double turnToThrirdBall = 0; //We might not need to tilt
                double driveToThirdBall = 155; //Might want to be closer depending on the reach of human player
                double thirdBallTilt = 0;
                double thirdBallTurret = 0;
                double thirdBallRPM = 0;
                double forwardToShootPoint = 0;
                // remaining data used in shoot routine

                addCommands(
                                new ParallelCommandGroup(
                                                new LimelightSetPipeline(ll, 1),
                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelRaceGroup(

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new TimeDelay(3),

                                                new RunActiveIntake(intake, transport))

                                                                .deadlineWith(new PositionHoldTiltTurret(
                                                                                tilt, turret,
                                                                                ll)),

                                new ParallelCommandGroup(

                                                new SetShootSpeedSource(shooter, shooter.fromPreset),

                                                new SetPresetRPM(shooter, upperRPM),

                                                new PositionTilt(tilt, upperTiltAngle),

                                                new PositionTurret(turret, upperTurretAngle)),

                                new ParallelRaceGroup(

                                                new SequentialCommandGroup(
                                                                new AltShootCargo(shooter, transport, intake,ll),

                                                                new AltShootCargo(shooter, transport, intake,ll)),

                                                new RunShooter(shooter))

                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll)),

                                new TurnToAngleProfiled(drive, turnToThrirdBall),

                                new ParallelCommandGroup(
                                                new LimelightSetPipeline(ll, 8),
                                                new SetShootSpeedSource(shooter, shooter.fromPreset),
                                                new SetPresetRPM(shooter, thirdBallRPM),
                                                new PositionStraight(drive, driveToThirdBall, positionRate),
                                                new PositionTurret(turret, thirdBallTurret),
                                                new PositionTilt(tilt, thirdBallTilt)),

                                new RunActiveIntake(intake, transport),

                                new PositionStraight(drive, forwardToShootPoint, positionRate), 

                                new LimelightSetPipeline(ll, 2),

                                new ParallelRaceGroup(
                                                new AltShootCargo(shooter, transport, intake,ll),
                                                new RunShooter(shooter))
                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll)),

                                new ParallelCommandGroup(new PositionTilt(tilt, 0), new PositionTurret(turret, 0)));

        }
}