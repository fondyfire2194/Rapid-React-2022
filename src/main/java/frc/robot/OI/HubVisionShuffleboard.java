// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.Set2XZoomValues;
import frc.robot.Vision.SetNoZoomValues;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.AutoCommands.Common.AcquireTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTestTarget;
import frc.robot.commands.AutoCommands.Common.GetNumberOfContourValues;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class HubVisionShuffleboard {


        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rcv2,
                        VisionReferenceTarget vrt, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter, boolean isMatch,
                        CargoTransportSubsystem transport, Compressor compressor) {

                // /**
                // *
                // * Vision
                // * && !isMatch
                // */

                if (true) {

                        ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("allXY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourPX.addString("LtoRTx", () -> rcv2.getLCRTx());
                        contourPX.addString("LtoRTy", () -> rcv2.getMedLCRTy());

                        contourPX.addString("LtoRTxMed", () -> rcv2.getMedLCRTx());
                        contourPX.addString("LtoRTyAngle", () -> rcv2.getLCRTyAngle());
                        contourPX.addString("LtoRTxMedAngle", () -> rcv2.getLCRTxMedAngle());

                        // contourPX.addNumber("AreaRatLR", () -> rcv2.getLRAreaRatio());
                        contourPX.addNumber("TX", () -> ll.get("tx"));
                        contourPX.addNumber("TY", () -> ll.get("ty"));

                        ShuffleboardLayout contourDist = Shuffleboard.getTab("HubVision")
                                        .getLayout("Dist", BuiltInLayouts.kList).withPosition(1, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        // contourDist.add("GetVisionData", new GetVisionValues(rcv2));

                        contourDist.add("Get6", new GetNumberOfContourValues(rcv2));

                        contourDist.add("CalcTestTarget", new CalculateTestTarget(vrt));

                        contourDist.add("CalcTarget", new CalculateTarget(rcv2));

                        contourDist.add("SetNoZoom", new SetNoZoomValues(rcv2, ll));

                        contourDist.add("Set2XZoom", new Set2XZoomValues(rcv2, ll));

                        contourDist.add("AcquireTarget",
                                        new AcquireTarget(ll, tilt, turret, rcv2, shooter, null, null, compressor));

                        contourDist.add("Calculate", new CalculateTargetDistance(ll, rcv2, tilt, turret, shooter));

                        contourPX.addString("LtoRMedArea", () -> rcv2.getLCRMedianArea());

                        ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("testX", BuiltInLayouts.kList).withPosition(3, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); //

                        testContourPX.addNumber("Test Con#", () -> vrt.getTestContourNumber());
                        testContourPX.addNumber("Test Tx", () -> vrt.getTestTargetTx());
                        testContourPX.addNumber("Test Ty", () -> vrt.getTestTargetTy());
                        testContourPX.addNumber("Test Tx Angle", () -> vrt.getTestTxAngle());
                        testContourPX.addNumber("Test Ty Angle", () -> vrt.getTestTyAngle());

                        testContourPX.addNumber("Test Area", () -> vrt.getTestTargetArea());

                }
                // &&!isMatch
                if (true) {
                        ShuffleboardLayout targetValues = Shuffleboard.getTab("HubVision")
                                        .getLayout("bullseye", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels)

                        targetValues.addNumber("AreaX", () -> rcv2.targetValue);
                        targetValues.addNumber("AreaAngle", () -> rcv2.targetAngle);

                        targetValues.addNumber("WeightedX", () -> rcv2.weightedTargetValue);
                        targetValues.addNumber("WeightedAngle", () -> rcv2.weightedTargetAngle);

                }

                // if (RobotBase.isReal()){// && !isMatch) {
                HttpCamera llght = new HttpCamera("CoprocessorCamera",
                                "http://10.21.94.11:5800/stream.mjpg");
                // CameraServer.addCamera(llght);

                ShuffleboardTab llFeed = Shuffleboard.getTab("HubVision");

                llFeed.add("Limelight_90", llght).withWidget(BuiltInWidgets.kCameraStream)
                                .withPosition(5, 0).withSize(3, 3)
                                .withProperties(Map.of("Show Crosshair", true,
                                                "Show Controls", true, "Rotation", "QUARTER_CW"));

        }

}
