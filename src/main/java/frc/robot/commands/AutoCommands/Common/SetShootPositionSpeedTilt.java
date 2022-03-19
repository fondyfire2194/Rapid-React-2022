// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldMap;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShootPositionSpeedTilt extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private RevTiltSubsystem m_tilt;
  private LimeLight m_ll;
  private int m_shootMode;

  public SetShootPositionSpeedTilt(RevShooterSubsystem shooter, RevTiltSubsystem tilt,LimeLight ll, int shootMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tilt = tilt;
    m_ll=ll;
    m_shootMode = shootMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooter.shootSpeedSource = m_shooter.fromPreset;
    m_shooter.presetModeName = FieldMap.shootModeName[m_shootMode];

    switch (m_shootMode) {

      case 0:
        m_shooter.presetRPM = FieldMap.shootMode_0[0];
        m_tilt.presetPosition = FieldMap.shootMode_0[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      case 1:
        m_shooter.presetRPM = FieldMap.shootMode_1[0];
        m_tilt.presetPosition = FieldMap.shootMode_1[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      case 2:
        m_shooter.presetRPM = FieldMap.shootMode_2[0];
        m_tilt.presetPosition = FieldMap.shootMode_2[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      case 3:
        m_shooter.presetRPM = FieldMap.shootMode_3[0];
        m_tilt.presetPosition = FieldMap.shootMode_3[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      default:
        m_shooter.presetRPM = FieldMap.shootMode_0[0];
        m_tilt.presetPosition = FieldMap.shootMode_0[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;

    }

  }
}
