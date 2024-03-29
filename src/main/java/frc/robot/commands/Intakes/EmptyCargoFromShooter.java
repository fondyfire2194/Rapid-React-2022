// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.CargoTransport.RunLowerRoller;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.RunTopRoller;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmptyCargoFromShooter extends ParallelRaceGroup {
  /** Creates a new EmptyCargoFromShooter. */
  public EmptyCargoFromShooter(RevShooterSubsystem shooter,
      CargoTransportSubsystem transport) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new JogShooter(shooter, () -> 0.25),
        new RunTopRoller(shooter, 800),
        new RunLowerRoller(transport, 800),
        new TimeDelay(3));
  }
}
