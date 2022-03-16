// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CargoTransport.RunLowerRollerIntake;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.ShootOneCargo;
import frc.robot.commands.Shooter.ShootTwoCargo;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

public class LeftRetPuAdv extends SequentialCommandGroup {

        // double intakeSpeed = Pref.getPref("IntakeSpeed");
        double pickUpRate = .2;// Pref.getPref("dRPur");

        /** Creates a new LRetPuShoot. */
        public LeftRetPuAdv(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter) {
                // Use addRequirements() here to declare subsystem dependencies.

                double drivePickupPosition = Units.inchesToMeters(-36);
                double shootPosition = Units.inchesToMeters(36);

                addCommands(

                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ActiveIntakeArmLower(intake),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelRaceGroup(

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new RunActiveIntake(intake, transport)),

                                new PositionStraight(drive, shootPosition, .3),

                                new SetShootSpeedSource(shooter, 2),

                                new SetPresetRPM(shooter, 1200),

                                new RunShooter(shooter),

                                new ShootTwoCargo(shooter, transport, intake)

                );

        }

}
