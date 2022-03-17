// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.ShootOneCargo;
import frc.robot.commands.Shooter.ShootTwoCargo;
//import frc.robot.commands.Shooter.ShootTwoCargo;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class UpperShoot extends SequentialCommandGroup {

  /** Creates a new LRetPuShoot. */
  public UpperShoot(RevTurretSubsystem turret, RevTiltSubsystem tilt,

      LimeLight ll, RevShooterSubsystem shooter,

      CargoTransportSubsystem transport, IntakesSubsystem intake, Compressor comp, double[] data) {

    // Use addRequirements() here to declare subsystem dependencies.
    // data 0 to 3 used i pickp and position routines
 
    double upperTiltPosition = data[2];
    double upperTurretPosition = data[3];
    double upperRPM = data[4];
    
    addCommands(

        new ParallelCommandGroup(new PositionTilt(tilt, upperTiltPosition),
 
        new PositionTurret(turret, upperTurretPosition)),

        new SetShootSpeedSource(shooter, shooter.fromPreset),

        new SetPresetRPM(shooter, upperRPM),

        new ParallelRaceGroup(new RunShooter(shooter),

            new ShootTwoCargo(shooter, transport, intake))

                .deadlineWith(new PositionHoldTiltTurret(tilt, turret, ll)));

  }

}
