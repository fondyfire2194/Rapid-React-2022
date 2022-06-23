// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
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
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CenterPuShootThirdCamera extends SequentialCommandGroup {

        private double firstAngle = 20;
        private double secondAngle = -20;
        private double thirdAngle = 0;

        /** Creates a new LRetPuShoot. */
        public CenterPuShootThirdCamera(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double positionRate = drive.positionRate;

                double drivePickupPosition = -1;

                double shootPosition = 2;

                // double upperRPM = data[4];
                // remaining data used in shoot routine

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new TurnToAngle(drive, firstAngle)
                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new ResetEncoders(drive),

                                new ResetGyro(drive),

                                new PositionStraight(drive, drivePickupPosition, positionRate)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                ll)),

                                new TurnToAngle(drive, secondAngle)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new ResetEncoders(drive),

                                new ResetGyro(drive),

                                new RunActiveIntake(intake, transport),

                                new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720, true),

                                new SetShootSpeedSource(shooter, shooter.cameraSource),
                                new TurnToAngle(drive, thirdAngle)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new ResetEncoders(drive),

                                new ResetGyro(drive),

                                new PositionStraight(drive, shootPosition, positionRate)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new WaitCommand(.25),

                                new SetUpCameraShoot(shooter, tilt, ll),

                                new ParallelRaceGroup(

                                                new SequentialCommandGroup(

                                                                new WaitCommand(.2),

                                                                new AltShootCargo(
                                                                                shooter,
                                                                                transport,
                                                                                intake,
                                                                                ll),
                                                                new WaitCommand(1)),

                                                new RunShooter(shooter)

                                                                .deadlineWith(new PositionHoldTiltTurret(
                                                                                tilt,
                                                                                turret,
                                                                                ll))),

                                new PositionTurret(turret, 0));

        }
}