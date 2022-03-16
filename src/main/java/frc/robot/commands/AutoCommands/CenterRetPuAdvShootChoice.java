// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.Common.LowerShoot;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.AutoCommands.Common.UpperShoot;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CenterRetPuAdvShootChoice extends SequentialCommandGroup {


        /** Creates a new LRetPuShoot. */
        public CenterRetPuAdvShootChoice(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll, Compressor comp, double[] data, boolean upper) {
 
                                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;
                double positionRate = drive.positionRate;

                double drivePickupPosition = data[0];
                double shootPosition = data[1];

                // remaining data used in shoot routine

                addCommands(

                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ActiveIntakeArmLower(intake),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelRaceGroup(

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new RunActiveIntake(intake, transport))

                                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,
                                                                                ll)),

                                new PositionStraight(drive, shootPosition, positionRate)

                                                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)),

                                new ConditionalCommand(
                                                new UpperShoot(turret, tilt, ll, shooter, transport, intake, comp,
                                                                data),
                                                new LowerShoot(turret, tilt, ll, shooter, transport, intake, comp,
                                                                data),
                                                () -> upper));

        }
}