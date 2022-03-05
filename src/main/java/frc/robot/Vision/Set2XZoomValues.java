// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Set2XZoomValues extends InstantCommand {

  private RawContoursV2 m_rcv2;
  private LimeLight m_ll;

  public Set2XZoomValues(RawContoursV2 rcv2, LimeLight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rcv2 = rcv2;
    m_ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ll.setPipeline(5);

    if (m_rcv2.cameraAt90) {

      m_rcv2.active_IMG_WIDTH = m_rcv2.ZOOM_IMG_HEIGHT;
      m_rcv2.active_IMG_WIDTH = m_rcv2.ZOOM_IMG_WIDTH;

      m_rcv2.activeHFOV = m_rcv2.ZOOM_CAMERA_VFOV;
      m_rcv2.activeVFOV = m_rcv2.ZOOM_CAMERA_HFOV;

    } else {
      
      m_rcv2.active_IMG_WIDTH = m_rcv2.ZOOM_IMG_WIDTH;
      m_rcv2.active_IMG_WIDTH = m_rcv2.ZOOM_IMG_HEIGHT;

      m_rcv2.activeHFOV = m_rcv2.ZOOM_CAMERA_VFOV;
      m_rcv2.activeVFOV = m_rcv2.ZOOM_CAMERA_HFOV;
    }

    m_rcv2.active_vpw = 2 * Math.tan(Units.degreesToRadians(m_rcv2.activeHFOV / 2));

    m_rcv2.active_vph = 2 * Math.tan(Units.degreesToRadians(m_rcv2.activeVFOV / 2));

  }
}
