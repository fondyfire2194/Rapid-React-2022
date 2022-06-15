// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.FieldMap;
import frc.robot.Pref;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPresetShootPositionSpeedTilt extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private RevTiltSubsystem m_tilt;
  private LimeLight m_ll;
  private int m_shootLocation;

  public SetPresetShootPositionSpeedTilt(RevShooterSubsystem shooter, RevTiltSubsystem tilt, LimeLight ll,
      int shootLocation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tilt = tilt;
    m_ll = ll;
    m_shootLocation = shootLocation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shootLocation = m_shootLocation;
    m_shooter.presetLocationName = FieldMap.shootLocationName[m_shooter.shootLocation];
    m_shooter.shootModeName = FieldMap.shootModeName[m_shooter.shootValuesSource];
    switch (m_shooter.shootLocation) {
      // at hub
      case 0:

        m_shooter.presetRPM = Pref.getPref("teleHubRPM");

        m_tilt.presetPosition = Pref.getPref("teleHubTilt");

        // m_shooter.presetRPM = FieldMap.shootLocation_0[1];
        // m_tilt.presetPosition = FieldMap.shootLocation_0[0];

        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        m_ll.setLEDMode(LedMode.kforceOff);

        break;

      // tarmac line
      case 1:

        m_shooter.presetRPM = Pref.getPref("teleTarRPM");

        m_tilt.presetPosition = Pref.getPref("teleTarTilt");

        // m_shooter.presetRPM = FieldMap.shootLocation_1[1];
        // m_tilt.presetPosition = FieldMap.shootLocation_1[0];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        m_ll.setLEDMode(LedMode.kpipeLine);

        break;
      case 2:
        m_shooter.presetRPM = FieldMap.shootLocation_2[1];
        m_tilt.presetPosition = FieldMap.shootLocation_2[0];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      case 3:// test mode
        m_shooter.shootValuesSource = m_shooter.fromThrottle;
        m_tilt.presetPosition = FieldMap.shootLocation_3[0];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;
      default:
        m_shooter.shootValuesSource = m_shooter.fromThrottle;
        m_shooter.presetRPM = m_shooter.getRPMfromThrottle();
        m_tilt.presetPosition = FieldMap.shootLocation_0[1];
        m_ll.setPipeline(PipelinesConstants.noZoom960720);
        break;

    }

  }
}
