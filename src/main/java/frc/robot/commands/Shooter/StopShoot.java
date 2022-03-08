/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StopShoot extends InstantCommand {
  private final RevShooterSubsystem m_shooter;

  private final CargoTransportSubsystem m_transport;

  public StopShoot(RevShooterSubsystem shooter, CargoTransportSubsystem transport) {
    m_shooter = shooter;
    m_transport = transport;
    addRequirements(m_shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooter.stop();
    m_shooter.stopTopRoller();
    m_transport.stopLowerRoller();

  }
}
