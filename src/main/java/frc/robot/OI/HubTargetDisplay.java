// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;

/** Add your docs here. */
public class HubTargetDisplay implements Sendable {
        private LimeLight m_ll;
        private RawContoursV2 m_rcv2;

        public HubTargetDisplay(LimeLight ll, RawContoursV2 rcv2) {
                m_ll = ll;
                m_rcv2 = rcv2;
        }

        @Override
        public void initSendable(SendableBuilder builder) {

                builder.setSmartDashboardType("HubTargetTrack");

                builder.addBooleanProperty("ll_has_target", m_ll::getIsTargetFound, null);

                builder.addDoubleArrayProperty("ty_median", m_rcv2::getMedianTY, null);

                builder.addDoubleArrayProperty("tx_median", m_rcv2::getMedianTX, null);

                builder.addDoubleArrayProperty("ty_medianVP", m_rcv2::getTyVpAngles, null);

                builder.addDoubleArrayProperty("tx_medianVP", m_rcv2::getTxVpAngles, null);

                builder.addDoubleArrayProperty("areas_0_1_2", m_rcv2::getAreaData, null);
 
                builder.addDoubleArrayProperty("areas_median", m_rcv2::getMedianAreas, null);               

                builder.addDoubleProperty("weightedX", m_rcv2::getWeightedX, null);

                builder.addDoubleProperty("weightedAngle", m_rcv2::getWeightedAngle, null);


        }
}
