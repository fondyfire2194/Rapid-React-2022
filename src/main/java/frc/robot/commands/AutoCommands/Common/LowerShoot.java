// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Vision.LimeLight;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LowerShoot extends SequentialCommandGroup {


  /** Creates a new LRetPuShoot. */
  public LowerShoot(RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight ll, RevShooterSubsystem shooter,
      CargoTransportSubsystem transport, Compressor comp, double[] data) {
    // Use addRequirements() here to declare subsystem dependencies.

    double lowerTiltAngle = data[4];
    double lowerTurretAngle = data[5];
    double lowerRPM = data[6];
    
    addCommands(

        new ParallelCommandGroup(new PositionTilt(tilt, lowerTiltAngle), new PositionTurret(turret, lowerTurretAngle)),

        new StartAllShooter(shooter, transport, lowerRPM),

        new ShootCargo(shooter, tilt, turret, ll, transport, comp, 6));

  }

}
