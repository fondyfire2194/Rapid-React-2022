// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SimpleCSVLogger;

/** Add your docs here. */
public class AngleSolver {

    private RawContoursV2 m_rcv2;

    private LimeLight m_ll;

    public double[] lrTxAngles = new double[3];

    public double[] lrTyAngles = new double[3];


    private double targetDistance;

    public boolean endLog;

    public SimpleCSVLogger hubLogger;

    public boolean lock3Contours;

    public boolean logInProgress;

    public double leftDistance;

    public double centerDistance;

    public double rightDistance;

    double[] lRTxVpValues = { 0, 0, 0 };

    double[] lRTyVpValues = { 0, 0, 0 };

    public AngleSolver(RawContoursV2 rcv2,LimeLight ll) {

        m_rcv2 = rcv2;

        hubLogger = new SimpleCSVLogger();
    }

    /**
     * Angle calculations are based on the LimeLight documentation
     * 
     * See the section on turning pixels into angles
     * 
     * Angles are used by the tilt and turret to move to and lock on target
     * 
     * 
     * 
     * @return
     */
    public void runValues() {

        lrTxAngles = getTxVpAngles(m_rcv2.lTRIndex);

        lrTyAngles = getTyVpAngles(m_rcv2.lTRIndex);


    }

    public void getlrtxData() {

        lRTxVpValues = getlrtxvp(m_rcv2.lTRIndex);

        lRTyVpValues = getlrtyvp(m_rcv2.lTRIndex);

    }

    private double[] getlrtyvp(int[] ltoRIndex) {

        double[] temp = { 0, 0, 0 };
        double vph2 = m_rcv2.vph / 2;
        temp[0] = vph2 * m_ll.get(m_rcv2.vertCoord + String.valueOf(ltoRIndex[0]));
        temp[1] = vph2 * m_ll.get(m_rcv2.vertCoord + String.valueOf(ltoRIndex[1]));
        temp[2] = vph2 * m_ll.get(m_rcv2.vertCoord + String.valueOf(ltoRIndex[2]));

        return temp;

    }

    private double[] getlrtxvp(int[] ltoRIndex) {

        double[] temp = { 0, 0, 0 };
        double vpw2 = m_rcv2.vpw / 2;

        temp[0] = vpw2 * m_ll.get(m_rcv2.horCoord + String.valueOf(ltoRIndex[0]));
        temp[1] = vpw2 * m_ll.get(m_rcv2.horCoord + String.valueOf(ltoRIndex[1]));
        temp[2] = vpw2 * m_ll.get(m_rcv2.horCoord + String.valueOf(ltoRIndex[2]));

        return temp;

    }



    public double[] getTxVpAngles(int[] index) {
        int[] m_index = index;
        double[] temp = { 0, 0, 0 };
        double x = 0;
        double ax = 0;// rads

        x = lRTxVpValues[m_index[0]];

        ax = Math.atan2(1, x);

        temp[0] = Units.radiansToDegrees(ax);

        x = lRTxVpValues[m_index[1]];

        ax = Math.atan2(1, x);

        temp[1] = Units.radiansToDegrees(ax);

        x = lRTxVpValues[m_index[2]];

        ax = Math.atan2(1, x);

        temp[2] = Units.radiansToDegrees(ax);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;
        temp[1] = -Math.round(temp[1] * 1000) / 1000.;
        temp[2] = -Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    public double[] getTyVpAngles(int[] index) {
        int[] m_index = index;
        double[] temp = { 0, 0, 0 };
        double y = 0;
        double ay = 0;// rads

        y = lRTyVpValues[m_index[0]];

        ay = Math.atan2(1, y);

        temp[0] = Units.radiansToDegrees(ay);

        y = lRTyVpValues[m_index[1]];

        ay = Math.atan2(1, y);

        temp[1] = Units.radiansToDegrees(ay);

        y = lRTyVpValues[m_index[2]];

        ay = Math.atan2(1, y);

        temp[2] = Units.radiansToDegrees(ay);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;
        temp[1] = -Math.round(temp[1] * 1000) / 1000.;
        temp[2] = -Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    
    public double getLeftArea() {
        return m_rcv2.getLeftArea();
    }

    public double getCenterArea() {
        return m_rcv2.getCenterArea();
    }

    public double getRightArea() {
        return m_rcv2.getRightArea();
    }

   

    // public void setCLock3Contours(boolean on) {
    //     m_rcv2.setLockContours(on);
    // }

   

    
}
