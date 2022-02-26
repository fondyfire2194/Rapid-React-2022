// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTarget;
import frc.robot.commands.AutoCommands.Common.CalculateTestTarget;
import frc.robot.commands.AutoCommands.Common.GetNumberOfContourValues;
import frc.robot.commands.AutoCommands.Common.GetVisionValues;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class HubVisionShuffleboard {

        private HttpCamera LLFeed;

        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2,
                        VisionReferenceTarget vrt, RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter, boolean isMatch) {

                // /**
                // *
                // * Vision
                // * && !isMatch
                // */

                if (true) {

                        ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("allXY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourPX.addString("LtoRTx", () -> rCV2.getLCRTx());
                        contourPX.addString("LtoRTy", () -> rCV2.getLCRTy());
 
                        contourPX.addString("LtoRTxMed", () -> rCV2.getMedLCRTx());
                        contourPX.addString("LtoRTyAngle", () -> rCV2.getLCRTyAngle());
                        contourPX.addString("LtoRTxMedAngle", () -> rCV2.getLCRTxMedAngle());
                        contourPX.addString("LtoRShort", () -> rCV2.getLCRShortSide());
                        contourPX.addString("LtoRLong", () -> rCV2.getLCRLongSide());
                        contourPX.addString("LtoRSkew", () -> rCV2.getLCRSkew());      

                        // contourPX.addNumber("AreaRatLR", () -> rCV2.getLRAreaRatio());
                        contourPX.addNumber("TX", () -> ll.get("tx"));
                        contourPX.addNumber("TY", () -> ll.get("ty"));

        
                        ShuffleboardLayout contourDist = Shuffleboard.getTab("HubVision")
                                        .getLayout("Dist", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourDist.add("GetVisionData", new GetVisionValues(rCV2));
                        contourDist.add("Get11", new GetNumberOfContourValues(rCV2));

                        contourDist.add("CalcTestTarget", new CalculateTestTarget(vrt));
                        contourDist.add("CalcTarget", new CalculateTarget(rCV2));

                        contourPX.addString("LtoRMedArea", () -> rCV2.getLCRMedianArea());

                        ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("testX", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

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
                                        .getLayout("bullseye", BuiltInLayouts.kList).withPosition(8, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels)

                        // targetValues.add("StartLog",
                        // new LogHubTarget(as, getTarget, tilt, turret, ll));
                        // targetValues.add("StopLog", new EndHubLog(as));

                        targetValues.addNumber("AreaAngle", () -> rCV2.targetAngle);
                        targetValues.addNumber("AreaX", () -> rCV2.targetValue);
                        targetValues.addNumber("WeightedX", () -> rCV2.weightedTargetValue);
                        targetValues.addNumber("WeightedAngle", () -> rCV2.weightedTargetAngle);
                        targetValues.addNumber("TargetAngle2", () -> rCV2.targetAngle2);

                }

                if (RobotBase.isReal() && !isMatch) {

                        LLFeed = new HttpCamera("limelight",
                                        "http://limelight.local:5800/stream.mjpg");
                        ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("HubVision");
                        driverDisplayTab.add("Limelight", LLFeed)
                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 2)
                                        .withSize(4, 2).withProperties(Map.of("Show Crosshair", false,
                                                        "Show Controls", true));//

                }
        }

}
