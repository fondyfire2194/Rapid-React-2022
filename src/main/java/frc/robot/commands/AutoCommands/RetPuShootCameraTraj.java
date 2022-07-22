// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.CargoTransport.TestCargoColor;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
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

public class RetPuShootCameraTraj extends SequentialCommandGroup {

        /** Creates a new LRetPuShoot. */
        public RetPuShootCameraTraj(IntakesSubsystem intake, RevDrivetrain drive, FondyFireTrajectory ff,
                        Trajectory traj,
                        CargoTransportSubsystem transport, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
                        RevTurretSubsystem turret, LimeLight ll, Compressor comp, double[] data) {
                addRequirements(intake, drive, transport, shooter, turret, tilt);
                // Use addRequirements() here to declare subsystem dependencies.

                double pickUpRate = drive.pickUpRate;
                // double positionRate = drive.positionRate;

                double drivePickupPosition = data[0];
                // double shootPosition = data[1];
                double tiltAngle = data[2];
                // double upperTurretAngle = data[3];
                double upperRPM = data[4];
                // remaining data used in shoot routine

                double timeOut = 15;

                if (RobotBase.isSimulation())
                        timeOut = 1;

                addCommands(
                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                // new ResetEncoders(drive),
                                                // new ResetGyro(drive),
                                                new SetShootSpeedSource(shooter, shooter.fromPreset),

                                                new SetPresetRPM(shooter, upperRPM),

                                                new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720,
                                                                true)),

                                new ParallelCommandGroup(
                                                // new PositionTilt(tilt, tiltAngle).withTimeout(timeOut),
                                                new PositionTilt(tilt, tiltAngle).withTimeout(timeOut),
                                                new WaitCommand(2),

                                                new RunActiveIntake(
                                                                intake, transport).withTimeout(timeOut),

                                                new SequentialCommandGroup(

                                                                new WaitCommand(.25),

                                                                new RunTrajectory(ff, drive, traj))),

                               

                                new RunShooter(shooter).raceWith(

                                                new SequentialCommandGroup(

                                                                // new WaitForTiltTurretInPosition(tilt, turret)
                                                                //                 .withTimeout(timeOut),
                                                                new WaitCommand(.4),
                                                                new AltShootCargo(
                                                                                shooter,
                                                                                transport,
                                                                                intake,
                                                                                ll).withTimeout(timeOut),

                                                                new WaitCommand(.2),

                                                                new TestCargoColor(transport, shooter).withTimeout(timeOut),

                                                                new WaitCommand(.2),
                                                            

                                                                new AltShootCargo(
                                                                                shooter,
                                                                                transport,
                                                                                intake,
                                                                                ll).withTimeout(timeOut),

                                                                new WaitCommand(1))),

                                new ParallelCommandGroup(

                                                new UseVision(ll, false),

                                                new LimelightSetPipeline(ll,
                                                                PipelinesConstants.ledsOffPipeline),

                                                new PositionTurret(
                                                                turret,
                                                                0).withTimeout(timeOut)));

        }
}