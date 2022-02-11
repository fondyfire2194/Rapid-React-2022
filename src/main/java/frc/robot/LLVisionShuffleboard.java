// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.Vision.LimelightCamMode;
import frc.robot.commands.Vision.LimelightLeds;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.LimelightStreamMode;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class LLVisionShuffleboard {

        private LimeLight m_limelight;

        private RawContoursV2 m_rCV2;
        private GetTarget m_getTarget;
        private RevTurretSubsystem m_turret;
        private RevTiltSubsystem m_tilt;
        private RevShooterSubsystem m_shooter;
        private HttpCamera LLFeed;

        private boolean m_showVision = true;

        private int loopCtr;

        public LLVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2, GetTarget getTarget, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter) {

                m_limelight = ll;
                m_rCV2 = rCV2;
                m_getTarget = getTarget;
                m_turret = turret;
                m_tilt = tilt;
                m_shooter = shooter;

                /**
                 * 
                 * 
                 * 
                 */

                if (m_showVision) {

                        ShuffleboardLayout zoomCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("Zoom", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //
                                                                                                         // labels //
                                                                                                         // for

                        zoomCommands.add("No Zoom",
                                        new LimelightSetPipeline(m_limelight, PipelinesConstants.noZoomPipeline));
                        zoomCommands.add("2XZoom",
                                        new LimelightSetPipeline(m_limelight, PipelinesConstants.x2ZoomPipeline));
                        zoomCommands.add("3X Zoom",
                                        new LimelightSetPipeline(m_limelight, PipelinesConstants.x3ZoomPipeline));

                        ShuffleboardLayout visionCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("On-Off", BuiltInLayouts.kList).withPosition(0, 2)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                        visionCommands.add("Vision On", new UseVision(m_limelight, true));
                        visionCommands.add("Vision Off", new UseVision(m_limelight, false));

                        ShuffleboardLayout cameraCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("Camera", BuiltInLayouts.kList).withPosition(1, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels
                                                                                                         // for

                        cameraCommands.add("DriverCam", new LimelightCamMode(m_limelight, CamMode.kdriver));
                        cameraCommands.add("VisionCam", new LimelightCamMode(m_limelight, CamMode.kvision));

                        cameraCommands.add("SideBySideStream",
                                        new LimelightStreamMode(m_limelight, StreamType.kStandard));
                        cameraCommands.add("MainPIP",
                                        new LimelightStreamMode(m_limelight, StreamType.kPiPMain));
                        cameraCommands.add("SecIP",
                                        new LimelightStreamMode(m_limelight, StreamType.kPiPSecondary));

                        ShuffleboardLayout ledCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(1, 3).withProperties(Map.of("Label position", "TOP"));

                        ledCommands.add("LedsOn", new LimelightLeds(m_limelight, LedMode.kforceOn));
                        ledCommands.add("LedsOff", new LimelightLeds(m_limelight, LedMode.kforceOff));
                        ledCommands.add("LedsBlink", new LimelightLeds(m_limelight, LedMode.kforceBlink));
                        ledCommands.add("LedsPipeline", new LimelightLeds(m_limelight, LedMode.kpipeLine));

                        ShuffleboardLayout visionData = Shuffleboard.getTab("LLVision")
                                        .getLayout("Data", BuiltInLayouts.kList).withPosition(3, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); //

                        visionData.addNumber("DegHToTarget", () -> m_limelight.getdegRotationToTarget());
                        visionData.addNumber("DegVertToTarget", () -> m_limelight.getdegVerticalToTarget());
                        visionData.addNumber("Pipeline #", () -> m_limelight.getPipeline());

                        visionData.addNumber("TargetArea", () -> m_limelight.getTargetArea());
                        visionData.addNumber("BNDBoxWidth", () -> m_limelight.getBoundingBoxWidth());
                        visionData.addNumber("BndBoxHeight", () -> m_limelight.getBoundingBoxHeight());
                        visionData.addNumber("CameraAngle", () -> m_tilt.getAngle());
                        visionData.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        visionData.addNumber("CameraCalculatedMPS", () -> m_shooter.cameraCalculatedSpeed);
                        visionData.addNumber("VertOff+Low", () -> m_limelight.verticalOffset);
                        visionData.addNumber("HorOff+Right", () -> m_limelight.horizontalOffset);

                        ShuffleboardLayout visionBools = Shuffleboard.getTab("LLVision")
                                        .getLayout("States", BuiltInLayouts.kList).withPosition(5, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels

                        visionBools.addBoolean("Connected", () -> m_limelight.isConnected());

                        visionBools.addBoolean("TargetVertOK",
                                        () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));

                        visionBools.addBoolean("TargetHorOK",
                                        () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));

                        visionBools.addBoolean("TargetFound", () -> m_limelight.getIsTargetFound());

                        visionBools.addBoolean("Use Vision", () -> m_limelight.useVision);

                }

        }

}
