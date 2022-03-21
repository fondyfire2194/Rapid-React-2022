// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class RetPuAdvShoot extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public RetPuAdvShoot(IntakesSubsystem intake, RevDrivetrain drive,
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

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelCommandGroup(

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new RunActiveIntake(intake, transport))
                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll)),

                                new ParallelCommandGroup(

                                                new SetShootSpeedSource(shooter, shooter.fromPreset),
                 
                                                new SetPresetRPM(shooter, upperRPM),

                                                new PositionStraight(drive, shootPosition, positionRate),
                 
                                                new PositionTilt(tilt, upperTiltAngle),
                 
                                                new PositionTurret(turret, upperTurretAngle)),

                                new ParallelRaceGroup(

                                                new SequentialCommandGroup(new ShootCargo(shooter, transport, intake),
                                                                
                                                new ShootCargo(shooter, transport, intake)),

                                                new RunShooter(shooter))

                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll)),

                                new ParallelCommandGroup(new PositionTilt(tilt, 0), new PositionTurret(turret, 0)));

        }
}