// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class GetCorners {

    private static int[][][] temp = new int[3][4][2];

    private static double[] myNewCornerData;

    private static boolean showAllData = true;

    private static int numberContoursFound;

    private static int length;

    public GetCorners() {

    }

    /**
     * 
     * Get corner data from tcornxy Network Table
     * For each target order is top left x, top left y, bottom left x bottom left y
     * ,
     * bottom right x, bottom right y, top right x bottom right y
     * 
     * Arrays are contours, corners, x y
     * 
     * With multiple corners the order needs to be sorted left to right to match the
     * hub targets
     * 
     * @return
     */
    public static int[][][] getCorners(double[] newCornerData) {
        myNewCornerData = newCornerData;
        int[][][] temp = new int[3][4][2];
        int x = 0;
        int y = 1;
        int cornersPerContour = 8;
        numberContoursFound = newCornerData.length / 8;
        if (numberContoursFound >= 1) {

            double[] first8 = Arrays.copyOfRange(myNewCornerData, 0, 8);
            SmartDashboard.putNumberArray("F18", first8);
        }

        if (numberContoursFound >= 2) {
            double[] scond8 = Arrays.copyOfRange(myNewCornerData, 8, 16);
            SmartDashboard.putNumberArray("F28", scond8);
        }

        if (numberContoursFound == 3) {
            double[] third8 = Arrays.copyOfRange(myNewCornerData, 17, 25);
            SmartDashboard.putNumberArray("F38", third8);
        }

        //
        //
        //

        length = myNewCornerData.length;

        if (showAllData) {
            SmartDashboard.putNumberArray("Pts", myNewCornerData);

        }

        /**
         * There are 3 contours so 3 sets of corners 0 to 2
         * Each corner has 4 xy point
         * Compound array is [contour number 0 to 3][corner position 0 to 3][x, y]
         * Array size is [4][4][2]
         * Incoming network table is (0)topLeft XY, (1)bottonLeft XY, (2)bottomRight XY,
         * (3)topRight XY
         * 
         */
        // x 0,2,4,6, 8,10,12,14, 16,18,20,22

        // y 1,3,5,7 9,11,13,15 17,19,21,23

        if (showAllData) {

            SmartDashboard.putNumber("ContoursFound", numberContoursFound);

            SmartDashboard.putNumber("ContoursFoundLength", myNewCornerData.length);
        }

        for (int contour = 0; contour < numberContoursFound; contour++) {

            for (int cornerNumber = 0; cornerNumber < 4; cornerNumber++) {

                int cornerPointer = contour * cornersPerContour;

                temp[contour][cornerNumber][x] = (int) myNewCornerData[cornerPointer + cornerNumber * 2];

                temp[contour][cornerNumber][y] = (int) myNewCornerData[cornerPointer + 1 + cornerNumber * 2];

            }

        }

        return temp;
    }

    public int[][][] getResult() {
        return temp;
    }

    public static int getContoursFound() {
        return numberContoursFound;
    }

    private void splitCornerData() {
        double[] firstInTable;
        double[] secondInTable;
        double[] thirdInTable;

        int found = getContoursFound();

        if (length >= 1) {
            firstInTable = Arrays.copyOfRange(myNewCornerData, 0, 7);
            SmartDashboard.putNumberArray("First", firstInTable);
        }

        if (found >= 2) {
            secondInTable = Arrays.copyOfRange(myNewCornerData, 8, 15);

            SmartDashboard.putNumberArray("Second", secondInTable);
        }
        if (length >= 3) {
            thirdInTable = Arrays.copyOfRange(myNewCornerData, 16, 23);

            SmartDashboard.putNumberArray("Third", thirdInTable);
        }
    }
}
