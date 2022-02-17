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
import frc.robot.Vision.AngleSolver;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Shooter.EndHubLog;
import frc.robot.commands.Shooter.LogHubTarget;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class HubVisionShuffleboard {

        private LimeLight m_ll;

        private RawContoursV2 m_rCV2;
        private GetTarget m_getTarget;
        private RevTurretSubsystem m_turret;
        private RevTiltSubsystem m_tilt;
        private AngleSolver m_as;
        private RevShooterSubsystem m_shooter;

        private HttpCamera LLFeed;

        public HubVisionShuffleboard(LimeLight ll, RawContoursV2 rCV2, AngleSolver as, GetTarget getTarget,
                        RevTurretSubsystem turret,
                        RevTiltSubsystem tilt, RevShooterSubsystem shooter) {

                m_ll = ll;
                m_rCV2 = rCV2;
                m_getTarget = getTarget;
                m_turret = turret;
                m_tilt = tilt;
                m_as = as;
                m_shooter = shooter;

                // /**
                // *
                // * Vision
                // *
                // */

                if (m_rCV2.lookForTarget || true) {

                        ShuffleboardLayout contourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("allXY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        contourPX.addString("LtoRTx", () -> m_rCV2.getLCRTx());
                        contourPX.addString("LtoRTy", () -> m_rCV2.getLCRTy());
                        contourPX.addString("LtoRTxAngle", () -> m_as.getLCRTxAngle());
                        contourPX.addString("LtoRTyAngle", () -> m_as.getLCRTyAngle());
                        contourPX.addString("LtoRArea", () -> m_rCV2.getLCRArea());
                        contourPX.addNumber("AreaRatLR", () -> m_rCV2.getLRAreaRatio());
                        contourPX.addNumber("TX", () -> m_ll.get("tx"));
                        contourPX.addNumber("TY", () -> m_ll.get("ty"));

                        ShuffleboardLayout contourDist = Shuffleboard.getTab("HubVision")
                                        .getLayout("Dist", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); //

                        contourDist.addNumber("LDist", () -> m_as.leftDistance);
                        contourDist.addNumber("CDist", () -> m_as.centerDistance);
                        contourDist.addNumber("RDist", () -> m_as.rightDistance);

                        ShuffleboardLayout testContourPX = Shuffleboard.getTab("HubVision")
                                        .getLayout("testX", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //

                        testContourPX.addNumber("Test Con#", () -> m_rCV2.getTestContourNumber());
                        testContourPX.addNumber("Test Tx", () -> m_rCV2.getTestTargetTx());
                        testContourPX.addNumber("Test Ty", () -> m_rCV2.getTestTargetTy());
                        testContourPX.addNumber("Test Tx Angle", () -> m_as.getTestTxAngle());
                        testContourPX.addNumber("Test Ty Angle", () -> m_as.getTestTyAngle());

                        testContourPX.addNumber("Test Area", () -> m_rCV2.getTestTargetArea());

                }

                if (m_getTarget.getTarget) {
                        ShuffleboardLayout targetValues = Shuffleboard.getTab("HubVision")
                                        .getLayout("bullseye", BuiltInLayouts.kList).withPosition(8, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels)

                        targetValues.add("StartLog",
                                        new LogHubTarget(m_as, m_getTarget, m_tilt, m_turret, m_ll));
                        targetValues.add("StopLog", new EndHubLog(m_as));

                        targetValues.addNumber("AreaAngle", () -> m_getTarget.getTargetAngle());
                        targetValues.addNumber("AreaX", () -> m_getTarget.getTargetX());

                }

                if (RobotBase.isReal()) {

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
