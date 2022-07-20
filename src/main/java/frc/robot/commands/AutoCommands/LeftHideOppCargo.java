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
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.RobotDrive.TurnToAngle;
import frc.robot.commands.Shooter.CheckCargoAtShoot;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.trajectories.CreateTrajectory;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.RotatePose;

/**
 * After shooting 2 cargo robot center is on alliance cargo center
 * 
 * Turn cw by ? and retract ? feet with rear intake to pick up opponent cargo
 * 
 * turn cw by ? and run cargo out of shooter
 * 
 * 
 */

public class LeftHideOppCargo extends SequentialCommandGroup {

        private double pickUpAngle = -135;

        final double pickupPosition = -1.1;

        private double shootAngle = -175;

        /** Creates a new LRetPuShoot. */
        public LeftHideOppCargo(IntakesSubsystem intake, RevDrivetrain drive,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, FondyFireTrajectory fftraj) {
                addRequirements(intake, drive, transport, shooter);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;

                double timeOut = 15;

                if (RobotBase.isSimulation())
                        timeOut = 1;

                addCommands(
                                // parallel(

                                new SetFrontIntakeActive(intake, false),
                                // new ResetEncoders(drive),
                                // new ResetGyro(drive),
                                new WaitCommand(.02),

                                new TurnToAngle(drive, pickUpAngle)

                                                .andThen(() -> drive.rotate(0)).withTimeout(timeOut),
                                sequence(
                                                new WaitCommand(1).deadlineWith(new StopRobot(drive),

                                                                new RotatePose(drive),

                                                                // new CreateTrajectory(drive, fftraj,
                                                                //                 fftraj.leftHideRev,
                                                                //                 fftraj.leftOppCargoRev, true),

                                                                new WaitCommand(.1))),

                                parallel(

                                                new IntakeToShootPosition(intake, transport)
                                                                .withTimeout(timeOut),

                                                fftraj.getRamsete(fftraj.leftHideRev).andThen(
                                                                () -> drive.tankDriveVolts(0, 0))),

                                new WaitCommand(.02),

                                new TurnToAngle(drive, shootAngle).andThen(() -> drive.stop()),

                                sequence(

                                                new WaitCommand(5).deadlineWith(new StopRobot(drive)),

                                                new RotatePose(drive)),

                                new RunCargoOutShooter(shooter, intake, transport, 700)

                                                .raceWith(
                                                                // new ResetEncoders(drive),
                                                                new WaitCommand(2)));

        }
}