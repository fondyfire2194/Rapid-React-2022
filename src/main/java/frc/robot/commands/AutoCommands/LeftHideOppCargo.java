// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

public class LeftHideOppCargo extends SequentialCommandGroup {

        private double pickUpAngle = 90;

        final double pickupPosition = -1;

        private double shootAngle = 10;

        /** Creates a new LRetPuShoot. */
        public LeftHideOppCargo(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter) {
                addRequirements(intake, drive, transport, shooter);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;


                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new TurnToAngle(drive, pickUpAngle),

                                new WaitCommand(.2),

                                new ResetEncoders(drive),

                                

                                new PositionStraight(drive, pickupPosition, pickUpRate)

                                                .deadlineWith(new RunActiveIntake(intake, transport)),

                                
                                new WaitCommand(.2),

                                new TurnToAngle(drive, shootAngle),

                                new WaitCommand(.2),

                                new ResetEncoders(drive),

                             

                                new ParallelRaceGroup(

                                                new RunCargoOutShooter(shooter, intake, transport, 700),

                                                new WaitCommand(3)));

        }
}