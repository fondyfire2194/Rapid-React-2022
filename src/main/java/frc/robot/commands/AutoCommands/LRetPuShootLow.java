// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.Common.AllRetPu;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LRetPuShootLow extends SequentialCommandGroup {
  /** Creates a new LRetPuShootLow. */
  public LRetPuShootLow(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret,
      RevTiltSubsystem tilt, double[] data) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(new AllRetPu(intake, drive, turret, tilt, data));

  }

}
