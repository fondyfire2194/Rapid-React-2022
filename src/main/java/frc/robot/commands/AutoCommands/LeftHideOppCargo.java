// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LeftHideOppCargo extends SequentialCommandGroup {

        private double pickUpAngle = 90;
        private double tiltAngle = 17;
        private double turretAngle = 50;

        final double pickupPosition = -1;
        final double returnDistance = pickupPosition;

        /** Creates a new LRetPuShoot. */
        public LeftHideOppCargo(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;
                double positionRate = drive.positionRate;

                // remaining data used in shoot routine

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new TurnToAngle(drive, pickUpAngle),

                                new ParallelCommandGroup(

                                                new PositionStraight(drive, pickupPosition,
                                                                pickUpRate),

                                                new PositionTilt(tilt, tiltAngle),

                                                new PositionTurret(turret, turretAngle),

                                                new TimeDelay(2))

                                                                .deadlineWith(new RunActiveIntake(intake, transport)),

                                new PositionStraight(drive, returnDistance, positionRate),

                                new ParallelRaceGroup(

                                                new RunCargoOutShooter(shooter, intake, transport)

                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll))),

                                new PositionTurret(turret, 0));

        }
}