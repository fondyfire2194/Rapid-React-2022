// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.TimedRobot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired
 * from the USB camera, then a rectangle is put on the image and sent to the
 * dashboard. OpenCV has
 * many methods for different types of processing.
 */
public class RotateLimelight90 extends TimedRobot {

    static Thread m_visionThread;
    static HttpCamera LLRFeed;

    public static void init() {
        m_visionThread = new Thread(
                () -> {

                    HttpCamera httpCamera = new HttpCamera("CoprocessorCamera",
                            "http://10.21.94.11:5800/video/stream.mjpg");
                    CameraServer.addCamera(httpCamera);
                    // Get a CvSink. This will capture Mats from the camera
                    CvSink cvSink = CameraServer.getVideo();
                    // Setup a CvSource. This will send images back to the Dashboard
                    CvSource outputStream = CameraServer.putVideo("Rotated", 320, 240);

                    // Mats are very memory expensive. Lets reuse this Mat.
                    Mat src = new Mat();

                    // This cannot be 'true'. The program will never exit if it is. This
                    // lets the robot stop this thread when restarting robot code or
                    // deploying.
                    while (!Thread.interrupted()) {
                        // Tell the CvSink to grab a frame from the camera and put it
                        // in the source mat. If there is an error notify the output.
                        if (cvSink.grabFrame(src) == 0) {
                            // Send the output the error.
                            outputStream.notifyError(cvSink.getError());
                            // skip the rest of the current iteration
                            continue;
                        }

                        // Rotate the Image

                        // Create empty Mat object to store output image
                        Mat dst = new Mat();

                        // Define Rotation Angle
                        double angle = 90;

                        // Image rotation according to the angle provided
                        if (angle == 90 || angle == -270)

                            Core.rotate(src, dst, Core.ROTATE_90_CLOCKWISE);
                        else if (angle == 180 || angle == -180)

                            Core.rotate(src, dst, Core.ROTATE_180);
                        else if (angle == 270 || angle == -90)

                            Core.rotate(src, dst,
                                    Core.ROTATE_90_CLOCKWISE);
                        else {

                            // Center of the rotation is given by
                            // midpoint of source image :
                            // (width/2.0,height/2.0)
                            Point rotPoint = new Point(src.cols() / 2.0,
                                    src.rows() / 2.0);

                            // Create Rotation Matrix
                            Mat rotMat = Imgproc.getRotationMatrix2D(
                                    rotPoint, angle, 1);

                            // Apply Affine Transformation
                            Imgproc.warpAffine(src, dst, rotMat, src.size(),
                                    Imgproc.WARP_INVERSE_MAP);
                            // Imgproc.warpAffine(src, dst, rotMat, src.size()
                            // );

                            // If counterclockwise rotation is required use
                            // following: Imgproc.warpAffine(src, dst,
                            // rotMat, src.size());
                        }

                        outputStream.putFrame(dst);
                    }
                });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }
}