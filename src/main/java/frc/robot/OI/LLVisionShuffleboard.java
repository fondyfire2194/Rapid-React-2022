// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.Vision.LimelightControlMode.StreamType;
import frc.robot.Vision.RawContoursV2;
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

        public static boolean m_showVision;

        HttpCamera LLFeed;

        public LLVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter) {

                /**
                 * 
                 * &&!isMatch
                 * 
                 */

                if (m_showVision) {

                        ShuffleboardLayout zoomCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("Zoom", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //
                                                                                                         // labels //
                                                                                                         // for

                        zoomCommands.add("No Zoom",
                                        new LimelightSetPipeline(ll, PipelinesConstants.noZoomPipeline));
                        zoomCommands.add("2XZoom",
                                        new LimelightSetPipeline(ll, PipelinesConstants.x2ZoomPipeline));
                        zoomCommands.add("3X Zoom",
                                        new LimelightSetPipeline(ll, PipelinesConstants.x3ZoomPipeline));

                        ShuffleboardLayout visionCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("On-Off", BuiltInLayouts.kList).withPosition(0, 2)
                                        .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                        visionCommands.add("Vision On", new UseVision(ll, true));
                        visionCommands.add("Vision Off", new UseVision(ll, false));

                        ShuffleboardLayout cameraCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("Camera", BuiltInLayouts.kList).withPosition(1, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels
                                                                                                         // for

                        cameraCommands.add("DriverCam", new LimelightCamMode(ll, CamMode.kdriver));
                        cameraCommands.add("VisionCam", new LimelightCamMode(ll, CamMode.kvision));

                        cameraCommands.add("SideBySideStream",
                                        new LimelightStreamMode(ll, StreamType.kStandard));
                        cameraCommands.add("MainPIP",
                                        new LimelightStreamMode(ll, StreamType.kPiPMain));
                        cameraCommands.add("SecIP",
                                        new LimelightStreamMode(ll, StreamType.kPiPSecondary));

                        ShuffleboardLayout ledCommands = Shuffleboard.getTab("LLVision")
                                        .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(1, 3).withProperties(Map.of("Label position", "TOP"));

                        ledCommands.add("LedsOn", new LimelightLeds(ll, LedMode.kforceOn));
                        ledCommands.add("LedsOff", new LimelightLeds(ll, LedMode.kforceOff));
                        ledCommands.add("LedsBlink", new LimelightLeds(ll, LedMode.kforceBlink));
                        ledCommands.add("LedsPipeline", new LimelightLeds(ll, LedMode.kpipeLine));

                        ShuffleboardLayout visionData = Shuffleboard.getTab("LLVision")
                                        .getLayout("Data", BuiltInLayouts.kList).withPosition(3, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); //

                        visionData.addNumber("DegHToTarget", () -> ll.getdegRotationToTarget());
                        visionData.addNumber("DegVertToTarget", () -> ll.getdegVerticalToTarget());
                        visionData.addNumber("Pipeline #", () -> ll.getPipeline());

                        visionData.addNumber("TargetArea", () -> ll.getTargetArea());
                        visionData.addNumber("BNDBoxWidth", () -> ll.getBoundingBoxWidth());
                        visionData.addNumber("BndBoxHeight", () -> ll.getBoundingBoxHeight());

                        visionData.addNumber("TargetDistance", () -> shooter.calculatedCameraDistance);

                        ShuffleboardLayout visionBools = Shuffleboard.getTab("LLVision")
                                        .getLayout("States", BuiltInLayouts.kList).withPosition(5, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels

                        visionBools.addBoolean("Connected", () -> ll.isConnected());

                        visionBools.addBoolean("TargetHorOK",
                                        () -> ll.getHorOnTarget(turret.turretVisionTolerance));

                        visionBools.addBoolean("TargetFound", () -> ll.getIsTargetFound());

                        visionBools.addBoolean("Use Vision", () -> ll.useVision);

                        
                }
        }

}
