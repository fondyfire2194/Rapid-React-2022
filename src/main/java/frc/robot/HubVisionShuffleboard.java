// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
public class HubVisionShuffleboard {

        private LimeLight m_limelight;

        private RawContoursV2 m_rCV2;
        private GetTarget m_getTarget;
        private RevTurretSubsystem m_turret;
        private RevTiltSubsystem m_tilt;
        private RevShooterSubsystem m_shooter;
        private HttpCamera LLFeed;

        private boolean m_showVision = true;

        private int loopCtr;

        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2, GetTarget getTarget, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter) {

                m_limelight = ll;
                m_rCV2 = rCV2;
                m_getTarget = getTarget;
                m_turret = turret;
                m_tilt = tilt;
                m_shooter = shooter;

                /**
                 * 
                 * Vision
                 * 
                 */

                if (m_showVision) {

                        ShuffleboardLayout zoomCommands = Shuffleboard.getTab("HubVision")
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

                        ShuffleboardLayout visionCommands = Shuffleboard.getTab("HubVision")
                                        .getLayout("On-Off", BuiltInLayouts.kList).withPosition(0, 2)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                        visionCommands.add("Vision On", new UseVision(m_limelight, true));
                        visionCommands.add("Vision Off", new UseVision(m_limelight, false));

                        ShuffleboardLayout cameraCommands = Shuffleboard.getTab("HubVision")
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

                        ShuffleboardLayout ledCommands = Shuffleboard.getTab("HubVision")
                                        .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(1, 3).withProperties(Map.of("Label position", "TOP"));

                        ledCommands.add("LedsOn", new LimelightLeds(m_limelight, LedMode.kforceOn));
                        ledCommands.add("LedsOff", new LimelightLeds(m_limelight, LedMode.kforceOff));
                        ledCommands.add("LedsBlink", new LimelightLeds(m_limelight, LedMode.kforceBlink));
                        ledCommands.add("LedsPipeline", new LimelightLeds(m_limelight, LedMode.kpipeLine));

                        ShuffleboardLayout visionData = Shuffleboard.getTab("HubVision")
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

                        if (m_rCV2.lookForTarget) {

                                ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                                .getLayout("allXY", BuiltInLayouts.kList).withPosition(5, 0)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                                contourPX.addString("LtoRTx", () -> m_rCV2.getLCRTx());
                                contourPX.addString("LtoRTy", () -> m_rCV2.getLCRTy());
                                contourPX.addString("LtoRArea", () -> m_rCV2.getLCRArea());
                                contourPX.addNumber("AreaRatR", () -> m_rCV2.getLRAreaRatio());
                         
                                ShuffleboardLayout contourLSSK = Shuffleboard.getTab("HubVision")
                                                .getLayout("LSSK", BuiltInLayouts.kList).withPosition(6, 0)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                                contourLSSK.addString("LeftLSSK", () -> m_rCV2.getLeftLSSk());
                                contourLSSK.addString("CenterLSSK", () -> m_rCV2.getCenterLSSk());
                                contourLSSK.addString("RightLSSK", () -> m_rCV2.getRightLSSk());

                                ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                                .getLayout("testX", BuiltInLayouts.kList).withPosition(7, 0)
                                                .withSize(1, 1).withProperties(Map.of("Label position", "TOP")); //

                                testContourPX.addNumber("Test Con#", () -> m_rCV2.getTestContourNumber());
                                testContourPX.addNumber("Test Tx", () -> m_rCV2.getTestTargetTx());
                                testContourPX.addNumber("Test Ty", () -> m_rCV2.getTestTargetTy());
                                testContourPX.addNumber("Test Area", () -> m_rCV2.getTestTargetArea());

                        }

                        if (m_getTarget.getTarget) {
                                ShuffleboardLayout targetValues = Shuffleboard.getTab("HubVision")
                                                .getLayout("bullseye", BuiltInLayouts.kList).withPosition(8, 0)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); // labels)

                                targetValues.addNumber("QuadX", () -> m_getTarget.getCenterX());
                                targetValues.addNumber("QuadY", () -> m_getTarget.getCenterY());
                                // targetValues.addNumber("avert", () -> m_getTarget.getAVert());
                                // targetValues.addNumber("bvert", () -> m_getTarget.getBVert());
                                // targetValues.addNumber("cvert", () -> m_getTarget.getCVert());
                                targetValues.addNumber("AreaTarget", () -> m_getTarget.getTargetX());
                        }

                        ShuffleboardLayout visionBools = Shuffleboard.getTab("HubVision")
                                        .getLayout("States", BuiltInLayouts.kList).withPosition(3, 3)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); // labels

                        visionBools.addBoolean("Connected", () -> m_limelight.isConnected());

                        visionBools.addBoolean("TargetVertOK",
                                        () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));

                        visionBools.addBoolean("TargetHorOK",
                                        () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));

                        visionBools.addBoolean("TargetFound", () -> m_limelight.getIsTargetFound());

                        visionBools.addBoolean("Use Vision", () -> m_limelight.useVision);

                        if (RobotBase.isReal()) {

                                LLFeed = new HttpCamera("limelight",
                                                "http://limelight.local:5800/stream.mjpg");
                                ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("HubVision");
                                driverDisplayTab.add("Limelight", LLFeed)
                                                .withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 2)
                                                .withSize(4, 2).withProperties(Map.of("Show Crosshair", true,
                                                                "Show Controls", false));//

                        }
                }

        }

}
