// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Vision.GetContourData;
import frc.robot.commands.Vision.GetContourTXValues;
import frc.robot.commands.Vision.GetTargetResults;
import frc.robot.subsystems.RevTurretSubsystem;

public class VerifyAndGetTarget extends SequentialCommandGroup {
  /** Creates a new VerifyAndGetTarget. */
  public VerifyAndGetTarget(RawContoursV2 rcv2, GetTarget target, RevTurretSubsystem turret) {
    
    addCommands(new GetContourData(rcv2), new GetContourTXValues(rcv2), new GetTargetResults(target, turret));
  
  }
}
