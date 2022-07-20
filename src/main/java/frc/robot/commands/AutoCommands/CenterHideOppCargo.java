// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoTransport.RunLowerRollerIntake;
import frc.robot.commands.Intakes.IntakeToShootPosition;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Shooter.CheckCargoAtShoot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.trajectories.CreateTrajectory;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.RotatePose;

public class CenterHideOppCargo extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public CenterHideOppCargo(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, FondyFireTrajectory fftraj,
                        Trajectory traj) {
                addRequirements(intake, drive, transport, shooter);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;

                final double pickUpAngle = 110;

                final double pickupPosition = 1.5;

                final double shootAngle = 132;

                double timeOut = 15;

                if (RobotBase.isSimulation())
                        timeOut = 1;

                // remaining data used in shoot routine

                addCommands(

                                new SetFrontIntakeActive(intake, true),

                                new TurnToAngle(drive, pickUpAngle).andThen(() -> drive.stop()),

                                new WaitCommand(.02),

                                new RotatePose(drive),

                                new WaitCommand(.02),

                                new CreateTrajectory(drive, fftraj, fftraj.centerHide, fftraj.centerHideOppCargo, false),

                                new WaitCommand(.02),

                                parallel(

                                                new IntakeToShootPosition(intake, transport).withTimeout(timeOut),

                                                fftraj.getRamsete(fftraj.centerHide).andThen(
                                                                () -> drive.tankDriveVolts(0, 0))),

                                new WaitCommand(.02),

                                race(
                                                new RunCargoOutShooter(shooter, intake, transport, 700),

                                                new WaitCommand(3)));

        }
}