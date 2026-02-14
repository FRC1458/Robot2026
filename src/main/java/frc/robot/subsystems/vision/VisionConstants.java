package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import frc.robot.Constants;

public class VisionConstants { 
    //TODO: this must be tuned to specific robot
    public static final double FOVHOR = 70; //degrees
    public static final double FOVVE = 48; //degrees
    public static final double CAMHEIGHT = 0.5; //meters or whatever length
    public static final double CAMANGLE = 0.0; //radians?
    public static final int OBSERVATION_BUFFER_SIZE = 50;
    public static final Matrix<N3, N1> STATE_STD_DEVS = 
        VecBuilder.fill(
            Math.pow(0.02, 1), 
            Math.pow(0.02, 1),
            Math.pow(0.02, 1)); // drive
    public static final Matrix<N3, N1> LOCAL_MEASUREMENT_STD_DEVS =
        VecBuilder.fill(
            Math.pow(0.2, 1), // vision
            Math.pow(0.2, 1),
            Math.pow(Double.POSITIVE_INFINITY, 1));
        
    public static enum VisionDeviceConstants {
        FR_CONSTANTS (
            "frontr",
            new Transform3d(
                new Translation3d(0.2822, 0.1087, 0.1984),
                new Rotation3d(0.5 * Constants.TAU, 14.0 * Constants.TAU / 360.0, -26.0 * Constants.TAU/360.0)),
            1, 1280, 800),
        
        FL_CONSTANTS (
            "frontl",
            new Transform3d(
                new Translation3d(0.2822, -0.1087, 0.1984),
                new Rotation3d(0.5 * Constants.TAU, 14.0 * Constants.TAU / 360.0, 26.0 * Constants.TAU/360.0)),
            2, 1280, 800);

        public final String tableName;
        public final Transform3d robotToCamera;
        public final int cameraId;
        public final int cameraResolutionWidth;
        public final int cameraResolutionHeight;
        private VisionDeviceConstants(
            String tableName, 
            Transform3d robotToCamera,
            int cameraId, 
            int cameraResolutionWidth,
            int cameraResolutionHeight
        ) {
            this.tableName = tableName;
            this.robotToCamera = robotToCamera;
            this.cameraId = cameraId;
            this.cameraResolutionWidth = cameraResolutionWidth;
            this.cameraResolutionHeight = cameraResolutionHeight;
        }
    }
}