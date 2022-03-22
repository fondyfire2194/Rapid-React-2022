// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToPreset;
import frc.robot.commands.Turret.PositionTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupForShootLocation extends SequentialCommandGroup {
  /** Creates a new SetupForShootLocation. */
  public SetupForShootLocation(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret,
      LimeLight ll, int shootMode) {
    {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        
          new SetShootPositionSpeedTilt(shooter, tilt, ll, shootMode),

          new PositionTilt(tilt, 0),
 
          new ParallelCommandGroup(

              new PositionTiltToPreset(tilt),

              new PositionTurret(turret, 0)));
    }
  }
}
