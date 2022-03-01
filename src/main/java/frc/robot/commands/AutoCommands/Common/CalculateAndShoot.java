// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalculateAndShoot extends SequentialCommandGroup {
  /** Creates a new CalculateAndShoot. */
  public CalculateAndShoot(RawContoursV2 rcv2, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight ll,
      RevShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new GetNumberOfContourValues(rcv2), new CalculateTarget(rcv2),

        new CalculateTargetDistance(ll, rcv2, tilt, turret, shooter),
        
        new CalculateSpeedAndTiltFromDistance(shooter, tilt)

    );
  }
}
