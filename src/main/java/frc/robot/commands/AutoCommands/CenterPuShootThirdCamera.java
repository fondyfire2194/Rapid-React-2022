// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterRangeConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.AutoCommands.Common.SetUpCameraShoot;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CenterPuShootThirdCamera extends SequentialCommandGroup {

        private double firstAngle;
        private double secondAngle;

        /** Creates a new LRetPuShoot. */
        public CenterPuShootThirdCamera(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double positionRate = drive.positionRate;

                double drivePickupPosition =-2;
                double shootPosition = -2;

                // double upperRPM = data[4];
                // remaining data used in shoot routine

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new TurnToAngle(drive, firstAngle)
                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new PositionStraight(drive, drivePickupPosition, positionRate)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                ll)),

                                new TurnToAngle(drive, secondAngle)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new RunActiveIntake(intake, transport),

                                new SetUpCameraShoot(shooter, tilt, ll),

                                new PositionStraight(drive, shootPosition, positionRate)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new ParallelCommandGroup(

                                                new ParallelRaceGroup(

                                                                new SequentialCommandGroup(
                                                                                new TimeDelay(.2),

                                                                                new AltShootCargo(
                                                                                                shooter,
                                                                                                transport,
                                                                                                intake,
                                                                                                ll),
                                                                                new TimeDelay(1)),

                                                                new RunShooter(shooter))

                                                                                .deadlineWith(new PositionHoldTiltTurret(
                                                                                                tilt,
                                                                                                turret,
                                                                                                ll)),

                                                new PositionTurret(turret, 0)));

        }
}