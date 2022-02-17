// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMap;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class AimAndLockTarget extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */
  private double tiltTargetPosition = FieldMap.tiltargetPosition;
  private double turretTargetPosition = FieldMap.turretargetPosition;
  

  public AimAndLockTarget(LimeLight ll, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      RawContoursV2 rcv2, GetTarget target) {
    addCommands(new PositionTiltToVision(tilt, ll, tiltTargetPosition),
        new PositionTurretToVision(turret, ll, turretTargetPosition),
        new ParallelCommandGroup(new PositionHoldTilt(tilt, ll), new PositionHoldTurret(turret, ll)),

        new VerifyAndGetTarget(rcv2, target, turret));

  }
}
