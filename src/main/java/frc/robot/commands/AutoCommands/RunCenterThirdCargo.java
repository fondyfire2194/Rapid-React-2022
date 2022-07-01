// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Shooter.AltShootCargo;
import frc.robot.commands.Shooter.CheckCargoAtShoot;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;
import frc.robot.trajectories.ResetOdometryToStartOfTrajectory;

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

    super(

        parallel(

            new ResetOdometryToStartOfTrajectory(drive, fftraj.centerThirdCargoPickUp),

            fftraj.getRamsete(fftraj.centerThirdCargoPickUp)
                .andThen(() -> drive.tankDriveVolts(0, 0)),
     
                new RunActiveIntake(intake, transport),
    
                new CheckCargoAtShoot(transport)),

        parallel(
          
        fftraj.getRamsete(fftraj.centerThirdCargoShoot)
            .andThen(() -> drive.tankDriveVolts(0, 0)),
        
            new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720, true),
        
            new SetPresetRPM(shooter, 888),
        
            new PositionTilt(tilt, 11)
                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll))),

        new AltShootCargo(shooter, transport, intake, ll)
            .deadlineWith(new RunShooter(shooter)));

  }
}
