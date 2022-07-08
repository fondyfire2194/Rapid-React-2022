// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.SaveGetSavedPose;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

public class CenterHideOppCargo extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public CenterHideOppCargo(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter) {
                addRequirements(intake, drive, transport, shooter);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;

                final double pickUpAngle = 90;

                final double pickupPosition = 1.2;

                final double shootAngle = 105;

                // remaining data used in shoot routine

                addCommands(

                                new SetFrontIntakeActive(intake, true),

                                new TurnToAngle(drive, pickUpAngle).andThen(() -> drive.stop()),

                                new WaitCommand(.02), // pose

                                new ResetEncoders(drive),

                                new WaitCommand(.02),

                                new ParallelCommandGroup(

                                                new WaitCommand(2),

                                                new PositionStraight(drive, pickupPosition, pickUpRate))

                                                                .deadlineWith(new RunActiveIntake(intake, transport)),

                                new WaitCommand(.02),

                                new TurnToAngle(drive, shootAngle).andThen(() -> drive.stop()),

                                new WaitCommand(.02),

                                race(
                                                new RunCargoOutShooter(shooter, intake, transport, 700),

                                                new WaitCommand(3)));

        }
}