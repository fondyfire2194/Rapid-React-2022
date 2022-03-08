// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHighFromUnderHub extends SequentialCommandGroup {
  /** Creates a new ShootHighFromUnderHub. */

  private double tiltAngle = 1;
  private double shootrpm = 1244;

  public ShootHighFromUnderHub(RevShooterSubsystem shooter, RevTiltSubsystem tilt, CargoTransportSubsystem transport,
      IntakesSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    shooter.presetRPM = 1244;

    addCommands(new ParallelCommandGroup(new PositionTilt(tilt, tiltAngle),

        new RunShooter(shooter, shooter.presetRPM)),

        new ShootCargo(shooter, 2, transport, intake));

  }
}
