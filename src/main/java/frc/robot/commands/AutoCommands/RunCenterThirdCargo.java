// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Constants.ShooterRangeConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Intakes.IntakeToShootPosition;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.RunTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunCenterThirdCargo extends SequentialCommandGroup {
        /** Creates a new RunCenterThirdTrajectory. */

        public RunCenterThirdCargo(RevDrivetrain drive, FondyFireTrajectory fftraj,
                        IntakesSubsystem intake, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, CargoTransportSubsystem transport, LimeLight ll) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());

                double timeOut = 15;

                if (RobotBase.isSimulation())
                        timeOut = 1;

                addCommands(

                                parallel(

                                                new PositionTilt(tilt,
                                                                ShooterRangeConstants.tiltRange3).withTimeout(timeOut),

                                                new RunTrajectory(fftraj, drive,
                                                                fftraj.centerThirdCargoPickUp),

                                                new SetUpLimelightForTarget(ll,
                                                                PipelinesConstants.noZoom960720,
                                                                true),

                                                new SetFrontIntakeActive(intake, false),

                                                new IntakeToShootPosition(intake, transport).withTimeout(timeOut)),

                                new SetUpLimelightForTarget(ll,
                                                PipelinesConstants.noZoom960720,
                                                true),

                                new RunShooter(shooter).raceWith(

                                                sequence(

                                                                new WaitCommand(2),

                                                                new AltShootCargo(
                                                                                shooter,
                                                                                transport,
                                                                                intake).withTimeout(timeOut),

                                                                new WaitCommand(.4))),

                                new ParallelCommandGroup(

                                                new UseVision(ll,
                                                                false),

                                                new LimelightSetPipeline(
                                                                ll,
                                                                PipelinesConstants.ledsOffPipeline),
                                                new PositionTurret(
                                                                turret,
                                                                0).withTimeout(timeOut)));

        }
}
