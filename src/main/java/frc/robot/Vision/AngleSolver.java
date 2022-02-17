// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import frc.robot.SimpleCSVLogger;

/** Add your docs here. */
public class AngleSolver {

    private RawContoursV2 m_rcv2;

    private LimeLight m_ll;

    private double HFOV = 30;// 59.6;
    private double VFOV = 23;// 45.7;
    double vpw = 2 * Math.tan(Units.degreesToRadians(HFOV / 2));
    double vph = 2 * Math.tan(Units.degreesToRadians(VFOV / 2));
    private double[] lrTxAngles = new double[3];

    private double[] lrTyAngles = new double[3];

    private double[] testAngles = { 0, 0 };

    private double HUB_VISION_DIAMETER = Units.inchesToMeters(53.375);
    private double HUB_CIRCUMFERENCE = Math.PI * HUB_VISION_DIAMETER;
    private double VISION_TAPE_LENGTH = Units.inchesToMeters(5);
    private double VISION_TAPE_WIDTH = Units.inchesToMeters(2);
    private double VISION_TAPE_GAP_LENGTH = Units.inchesToMeters(5.5);

    private double ANGLE_BETWEEN_SAME_POINTS_OF_VISION_TAPE = 360 / 16;

    private double VISION_TAPE_GAP_CONTAINED_ANGLE = VISION_TAPE_GAP_LENGTH * 360 / HUB_CIRCUMFERENCE;

    private double VISION_TAPE_WIDTH_CONTAINED_ANGLE = VISION_TAPE_LENGTH * 360 / HUB_CIRCUMFERENCE;

    private double VISION_TAPE_BOTTOM_HEIGHT = Units.inchesToMeters(101.675);

    private double CAMERA_HEIGHT = Units.inchesToMeters(30);

    private double CAMERA_ANGLE_TO_HORIZONTAL = 10;

    private double targetDistance;

    public boolean endLog;

    public SimpleCSVLogger hubLogger;

    public boolean lock3Contours;

    public boolean logInProgress;

    public double leftDistance;

    public double centerDistance;

    public double rightDistance;

   
    public AngleSolver(RawContoursV2 rcv2, LimeLight ll) {

        m_rcv2 = rcv2;
        m_ll = ll;
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

        lrTxAngles = getTxVpAngles();

        lrTyAngles = getTyVpAngles();

        testAngles = getTestTargetVPAngles();

        leftDistance = getTargetDistance(lrTyAngles[0]);
        centerDistance = getTargetDistance(lrTyAngles[1]);
        rightDistance = getTargetDistance(lrTyAngles[2]);

        
    }

    private double[] getTxVpAngles() {

        double[] temp = { 0, 0, 0 };
        int[] quadrant = { 0, 0, 0 };
        double x = 0;
        double ax = 0;// rads

        x = m_rcv2.lRTxVpValues[0];

        ax = Math.atan2(1, x);
        quadrant[0] = getQuadrant(ax);

        temp[0] = Units.radiansToDegrees(ax);

        x = m_rcv2.lRTxVpValues[1];

        ax = Math.atan2(1, x);
        quadrant[1] = getQuadrant(ax);
        temp[1] = Units.radiansToDegrees(ax);

        x = m_rcv2.lRTxVpValues[2];
        ax = Math.atan2(1, x);
        quadrant[2] = getQuadrant(ax);
        temp[2] = Units.radiansToDegrees(ax);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;
        temp[1] = -Math.round(temp[1] * 1000) / 1000.;
        temp[2] = -Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    private double[] getTyVpAngles() {

        double[] temp = { 0, 0, 0 };
        int[] quadrant = { 0, 0, 0 };
        double y = 0;
        double ay = 0;// rads

        y = m_rcv2.lRTyVpValues[0];

        ay = Math.atan2(1, y);

        temp[0] = Units.radiansToDegrees(ay);

        y = m_rcv2.lRTyVpValues[1];

        ay = Math.atan2(1, y);

        quadrant[0] = getQuadrant(ay);

        temp[1] = Units.radiansToDegrees(ay);

        quadrant[1] = getQuadrant(ay);

        y = m_rcv2.lRTyVpValues[2];

        ay = Math.atan2(1, y);

        quadrant[2] = getQuadrant(ay);

        temp[2] = Units.radiansToDegrees(ay);

        temp[0] -= 90;
        temp[1] -= 90;
        temp[2] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;
        temp[1] = -Math.round(temp[1] * 1000) / 1000.;
        temp[2] = -Math.round(temp[2] * 1000) / 1000.;

        return temp;
    }

    private double[] getTestTargetVPAngles() {

        double[] temp = { 0, 0 };
        int[] quadrant = { 0, 0 };
        double x = 0;
        double ax = 0;// rads

        x = m_rcv2.testTxVPValue;

        ax = Math.atan2(1, x);

        quadrant[0] = getQuadrant(ax);

        temp[0] = Units.radiansToDegrees(ax);

        double y = m_rcv2.testTyVPValue;

        double ay = Math.atan2(1, y);

        quadrant[1] = getQuadrant(ay);

        temp[1] = Units.radiansToDegrees(ay);

        temp[0] -= 90;

        temp[0] = -Math.round(temp[0] * 1000) / 1000.;

        temp[1] -= 90;

        temp[1] = -Math.round(temp[1] * 1000) / 1000.;

        return temp;
    }

    private int getQuadrant(double radVal) {
        int temp = 0;

        if (radVal >= 0 && radVal < Math.PI / 4) {
            temp = 1;
        }
        if (radVal >= Math.PI && radVal < Math.PI / 2) {
            temp = 2;
        }
        if (radVal >= Math.PI / 2 && radVal < 3 * Math.PI / 4) {
            temp = 3;
        }
        if (radVal >= 3 * Math.PI / 4 && radVal < 0) {
            temp = 4;
        }
        return temp;

    }

    private double[] getRawCrosshairVPAngles() {
        double[] temp = { 0, 0 };
        temp[0] = m_ll.get("cx0");
        temp[1] = m_ll.get("cy0");
        return temp;

    }

    // METERS
    private double getTargetDistance(double ty) {
        VISION_TAPE_BOTTOM_HEIGHT = Units.inchesToMeters(78);
        double distance = 0;
        double h1 = CAMERA_HEIGHT;
        double h2 = VISION_TAPE_BOTTOM_HEIGHT + VISION_TAPE_WIDTH;
        double a1 = Units.degreesToRadians((ty));
        double a2 = Units.degreesToRadians(CAMERA_ANGLE_TO_HORIZONTAL);

        distance = (h2 - h1) / Math.tan(a1 + a2);

        return distance;
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

    public double getTestTargetArea() {
        return m_rcv2.getTestTargetArea();
    }

    public void setCLock3Contours(boolean on) {
        m_rcv2.setLockContours(on);
    }

    public double getTestTxAngle() {
        return testAngles[0];
    }

    public double getTestTyAngle() {
        return testAngles[1];
    }

    public double getLeftTxAngle() {
        return lrTxAngles[0];
    }

    public double getCenterTxAngle() {
        return lrTxAngles[1];
    }

    public double getRightTxAngle() {
        return lrTxAngles[2];
    }

    public double getLeftTyAngle() {
        return lrTyAngles[0];
    }

    public double getCenterTyAngle() {
        return lrTyAngles[1];
    }

    public double getRightTyAngle() {
        return lrTyAngles[2];
    }

    public String getLCRTxAngle() {

        return String.valueOf(getLeftTxAngle() + " ," + String.valueOf(getCenterTxAngle()) + " ,"
                + String.valueOf(getRightTxAngle()));
    }

    public String getLCRTyAngle() {
        return String.valueOf(getLeftTyAngle() + " ," + String.valueOf(getCenterTyAngle()) + " ,"
                + String.valueOf(getRightTyAngle()));

    }

    public String getXYPoints(double[] points) {
        return String.valueOf(points[0]) + "," + String.valueOf(points[1]);
    }

    public double getTargetDistance() {
        return targetDistance;
    }

}
