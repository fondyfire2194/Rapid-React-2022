// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.ResetOdometryToStartOfTrajectory;

public class RunRightFirstPickup extends SequentialCommandGroup {
  /** Creates a new RunRightFirstPickup. */
  public RunRightFirstPickup(RevDrivetrain drive, FondyFireTrajectory fftraj,
      IntakesSubsystem intake, RevShooterSubsystem shooter, RevTiltSubsystem tilt,
      RevTurretSubsystem turret, CargoTransportSubsystem transport, LimeLight ll) {
    // Use addRequirements() here to declare subsystem dependencies.

    // Called when the command is initially scheduled.
    double timeOut = 15;

    double endPoint = 1.2;

    if (RobotBase.isSimulation())
      timeOut = 1;

    addCommands(

        parallel(
            new SetFrontIntakeActive(intake, true),

            new RunActiveIntake(intake, transport).withTimeout(timeOut),

            new ResetOdometryToStartOfTrajectory(fftraj,
                fftraj.rightFirstCargoPickup, drive),

            new WaitCommand(.1),

            fftraj.getRamsete(fftraj.rightFirstCargoPickup))
                .andThen(() -> drive.tankDriveVolts(0, 0)));
  }
}