// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RawContoursV2 {

    private int IMG_WIDTH = 320;
    private int IMG_HEIGHT = 240;

    private int[] areaIndex = { 0, 1, 2 };

    private boolean showDebug = true;

    public boolean lock3Contours = false;

    int[] areasLRTxIndex = { 0, 1, 2 };

    private double[] lToRAreas = { 0, 0, 0, 0 };

    private int[] lRTxValues = { 0, 0, 0 };

    private int[] lRTYValues = { 0, 0, 0 };

    private int maxPossibleContours = 6;

    public boolean lookForTarget = true;

    private LimeLight m_ll;

    private boolean cameraAt90 = false;

    String horCoord;

    String vertCoord;

    public int testTargetIndex;

    private int testTargetTx;

    private int testTargetTy;

    private double testTargetTa;

    public boolean testTargetInUse = true;

    private double[][] ltoRLongShortSkew = new double[3][3];

    public boolean endLog;

    public boolean logInProgress;

    public SimpleCSVLogger hubLogger;
    public boolean log;

    public RawContoursV2(LimeLight ll) {

        m_ll = ll;

        horCoord = "tx";
        vertCoord = "ty";
        IMG_WIDTH = 320;
        IMG_HEIGHT = 240;

        if (cameraAt90) {
            horCoord = "ty";
            vertCoord = "tx";
            IMG_WIDTH = 240;
            IMG_HEIGHT = 320;
        }

    }

    // center post target

    public void getRawContourData() {

        hubLogger = new SimpleCSVLogger();

        lookForTarget = m_ll.getIsTargetFound();
        lookForTarget = true;
        if (lookForTarget) {

            if (testTargetInUse) {

                testTargetIndex = getLowestContourIndex();

            }

            // pointers to 3 largest contours

            if (!lock3Contours) {

                areaIndex = getIndexOf3LargestContours();

                // pointers to LMR

                areasLRTxIndex = getLeftCenterRightTxPointer(areaIndex);

            }

            lRTxValues = getLRTXValues(areaIndex);

            lRTYValues = getLRTYValues(areasLRTxIndex);

            lToRAreas = get3LargestAreas(areaIndex);

            ltoRLongShortSkew = getLongShortSkew(areasLRTxIndex);

            displayData();

            if (showDebug)

                debugDisplay();
        }

    }

    private int[] getIndexOf3LargestContours() {
        double[] temp = { 0, 0, 0, 0, 0, 0 };

        int[] tempI = { 0, 1, 2, 3, 4, 5 };

        int[] tempI1 = { 0, 1, 2 };

        int swI;
        double swD;

        boolean swap = true;

        int range = temp.length - 1;

        for (int i = 0; i < maxPossibleContours; i++) {

            if (testTargetInUse && i == testTargetIndex) {

                continue;
            }

            temp[i] = m_ll.get("ta" + String.valueOf(i));
        }

        while (swap) {
            swap = false;
            for (int i = 0; i < range; i++) {
                if (temp[i] < temp[i + 1]) {
                    swD = temp[i + 1];
                    temp[i + 1] = temp[i];
                    temp[i] = swD;
                    swI = tempI[i + 1];
                    tempI[i + 1] = tempI[i];
                    tempI[i] = swI;
                    swap = true;
                }
            }
            range--;
        }

        tempI1 = Arrays.copyOfRange(tempI, 0, 3);

        return tempI1;
    }

    private double[] get3LargestAreas(int[] index) {

        double[] temp = { 0, 0, 0, 0 };

        temp[0] = m_ll.get("ta" + String.valueOf(index[0])) * 1000;

        temp[1] = m_ll.get("ta" + String.valueOf(index[1])) * 1000;

        temp[2] = m_ll.get("ta" + String.valueOf(index[2])) * 1000;

        temp[3] = temp[0] / temp[2];// ratio of outside areas
        temp[0] = Math.round(temp[0]);
        temp[1] = Math.round(temp[1]);
        temp[2] = Math.round(temp[2]);

        return temp;
    }

    private int[] getLeftCenterRightTxPointer(int[] areaIndex) {
        double[] contourTx = { 0, 0, 0 };
        int[] tempI = areaIndex;// Arrays.copyOfRange(areaIndex, 0, 3);
        double swD;
        int swI;
        contourTx[0] = (((1 + (m_ll.get(horCoord + String.valueOf(areaIndex[0])))) / 2) * IMG_WIDTH);
        contourTx[1] = (((1 + (m_ll.get(horCoord + String.valueOf(areaIndex[1])))) / 2) * IMG_WIDTH);
        contourTx[2] = (((1 + (m_ll.get(horCoord + String.valueOf(areaIndex[2])))) / 2) * IMG_WIDTH);

        boolean swap = true;
        int range = contourTx.length - 1;
        while (swap) {
            swap = false;
            for (int i = 0; i < range; i++) {
                if (contourTx[i] > contourTx[i + 1]) {
                    swD = contourTx[i + 1];
                    contourTx[i + 1] = contourTx[i];
                    contourTx[i] = swD;

                    swI = tempI[i + 1];
                    tempI[i + 1] = tempI[i];
                    tempI[i] = swI;
                    swap = true;
                }
            }
            range--;
        }
        return tempI;
    }

    private int[] getLRTXValues(int[] lToRIndex) {

        double[] temp = { 0, 0, 0 };
        int[] tempI = { 0, 0, 0 };

        temp[0] = (((1 + (m_ll.get(horCoord + String.valueOf(lToRIndex[0])))) / 2) * IMG_WIDTH);
        temp[1] = (((1 + (m_ll.get(horCoord + String.valueOf(lToRIndex[1])))) / 2) * IMG_WIDTH);
        temp[2] = (((1 + (m_ll.get(horCoord + String.valueOf(lToRIndex[2])))) / 2) * IMG_WIDTH);

        for (int i = 0; i < temp.length; i++)
            tempI[i] = (int) temp[i];

        return tempI;

    }

    private int[] getLRTYValues(int[] lToRIndex) {

        double[] temp = { 0, 0, 0 };
        int[] tempI = { 0, 0, 0 };

        temp[0] = (((1 - (m_ll.get(vertCoord + String.valueOf(lToRIndex[0])))) / 2) * IMG_HEIGHT);
        temp[1] = (((1 - (m_ll.get(vertCoord + String.valueOf(lToRIndex[1])))) / 2) * IMG_HEIGHT);
        temp[2] = (((1 - (m_ll.get(vertCoord + String.valueOf(lToRIndex[2])))) / 2) * IMG_HEIGHT);

        for (int i = 0; i < temp.length; i++)
            tempI[i] = (int) temp[i];

        return tempI;

    }

    private double[][] getLongShortSkew(int[] lToRIndex) {

        // get the long and short sides and skew of the fitted boundng boxes
        // [0] is long, [1] is short, [2] is skew

        double[][] temp = new double[3][3];

        for (int i = 0; i < 3; i++) {

            temp[i][0] = Math.round(m_ll.get("tlong" + String.valueOf(lToRIndex[i])));
            temp[i][1] = Math.round(m_ll.get("tshort" + String.valueOf(lToRIndex[i])));
            temp[i][2] = Math.round(m_ll.get("ts" + String.valueOf(lToRIndex[i])));

        }

        return temp;

    }

    private int getLowestContourIndex() {
        double temp = 0;
        double min = IMG_HEIGHT;
        int lowest = 0;
        int i = 0;

        for (i = 0; i < maxPossibleContours; i++) {
            temp = (((1 + (m_ll.get(vertCoord + String.valueOf(i)))) / 2) * IMG_WIDTH);
            if (temp < min) {
                min = temp;
                lowest = i;
            }
        }

        double tempx = (((1 + (m_ll.get(horCoord + String.valueOf(lowest)))) / 2) * IMG_WIDTH);
        double tempy = (((1 - (m_ll.get(vertCoord + String.valueOf(lowest)))) / 2) * IMG_HEIGHT);
        double tempa = Math.round((m_ll.get("ta" + String.valueOf(lowest)) / 2) * 100);
        SmartDashboard.putNumber("lowest", lowest);

        testTargetTx = (int) tempx;
        testTargetTy = (int) tempy;
        testTargetTa = (int) tempa;

        return lowest;
    }

    private double[] showAsDoubleArray(int[] values) {

        int[] temp = values;
        double[] tempA = new double[6];

        for (int i = 0; i < temp.length; i++) {
            tempA[i] = (double) temp[i];

        }
        double[] tempB = Arrays.copyOfRange(tempA, 0, temp.length);

        return tempB;

    }

    public double getLeftArea() {
        return lToRAreas[0];
    }

    public double getCenterArea() {
        return lToRAreas[1];
    }

    public double getRightArea() {
        return lToRAreas[2];
    }

    public double getLRAreaRatio() {
        return lToRAreas[3];
    }

    public int getLeftTx() {
        return lRTxValues[0];
    }

    public int getCenterTx() {
        return lRTxValues[1];
    }

    public int getRightTx() {
        return lRTxValues[2];
    }

    public int getTestContourNumber() {
        return testTargetIndex;
    }

    public int getTestTargetTx() {
        return testTargetTx;
    }

    public int getTestTargetTy() {
        return testTargetTy;
    }

    public double getTestTargetArea() {
        return testTargetTa;
    }

    public int getLeftTy() {
        return lRTYValues[0];
    }

    public int getCenterTy() {
        return lRTYValues[1];
    }

    public int getRightTy() {
        return lRTYValues[2];
    }

    public void setLockContours(boolean on) {
        lock3Contours = on;
    }

    public String getLCRTx() {

        return String.valueOf(getLeftTx() + " ," + String.valueOf(getCenterTx()) + " ," + String.valueOf(getRightTx()));
    }

    public String getLCRTy() {
        return String.valueOf(getLeftTy() + " ," + String.valueOf(getCenterTy()) + " ," + String.valueOf(getRightTy()));

    }

    public String getLCRArea() {

        return String.valueOf(
                getLeftArea() + " ," + String.valueOf(getCenterArea()) + " ," + String.valueOf(getRightArea()));
    }

    public String getLeftLSSk() {
        return String.valueOf(ltoRLongShortSkew[0][0] + " ," + String.valueOf(ltoRLongShortSkew[0][1]) + " ,"
                + String.valueOf(ltoRLongShortSkew[0][2]));
    }

    public String getCenterLSSk() {
        return String.valueOf(ltoRLongShortSkew[1][0] + " ," + String.valueOf(ltoRLongShortSkew[1][1]) + " ,"
                + String.valueOf(ltoRLongShortSkew[1][2]));
    }

    public String getRightLSSk() {
        return String.valueOf(ltoRLongShortSkew[2][0] + " ," + String.valueOf(ltoRLongShortSkew[2][1]) + " ,"
                + String.valueOf(ltoRLongShortSkew[2][2]));
    }

    boolean getLRAreasEqual(double tol) {
        return Math.abs(getLeftArea() - getRightArea()) < tol;
    }

    public boolean getLRTyValuesEqual(double tol) {
        return Math.abs(getLeftTy() - getRightTy()) < tol;
    }

    public boolean getTiltOnTarget(int tol) {
        int tiltAngle = 0;
        return Math.abs(tiltAngle - getCenterTy()) < tol;
    }

    public boolean getTurrettOnTarget(int tol) {
        int turretAngle = 0;
        return Math.abs(turretAngle - getCenterTx()) < tol;
    }

    public boolean OKToShoot() {
        int tol = 1;
        return getTurrettOnTarget(tol) && getTiltOnTarget(tol) && getLRAreasEqual(tol);
    }

    private void displayData() {

        SmartDashboard.putNumber("Left Area", getLeftArea());
        SmartDashboard.putNumber("Center Area", getCenterArea());
        SmartDashboard.putNumber("Right Area", getRightArea());

        SmartDashboard.putNumber("Left Tx", getLeftTx());
        SmartDashboard.putNumber("Center Tx", getCenterTx());
        SmartDashboard.putNumber("Right Tx", getRightTx());

        SmartDashboard.putNumber("Left Ty", getLeftTy());
        SmartDashboard.putNumber("Center Ty", getCenterTy());
        SmartDashboard.putNumber("Right Ty", getRightTy());

        SmartDashboard.putBoolean("LRAreasSame", getLRAreasEqual(1));
        SmartDashboard.putBoolean("LRTYsSame", getLRTyValuesEqual(5));

        SmartDashboard.putBoolean("TiltOnTarget", getTiltOnTarget(1));
        SmartDashboard.putBoolean("TurretOntargt", getTurrettOnTarget(1));
        SmartDashboard.putBoolean("OkToShoot", OKToShoot());

    }

    private double getnX(double x) {
        return ((x - 159.5)) / 160;
    }

    private double getnY(int y) {
        return (1 / (IMG_WIDTH / 2)) * (y - ((double) IMG_WIDTH) - .5);
    }

    // private double get(String varName) {
    // return
    // NetworkTableInstance.getDefault().getTable(mTableName).getEntry(varName).getDouble(0);
    // }

    private void debugDisplay() {

        double[] temp = showAsDoubleArray(areaIndex);
        SmartDashboard.putNumberArray("3LargeIndex", temp);

        double[] tempIndex = showAsDoubleArray(areasLRTxIndex);
        SmartDashboard.putNumberArray("LRAreasIndex", tempIndex);

        tempIndex = showAsDoubleArray(areasLRTxIndex);
        SmartDashboard.putNumberArray("LRTXIndex", tempIndex);

    }

}