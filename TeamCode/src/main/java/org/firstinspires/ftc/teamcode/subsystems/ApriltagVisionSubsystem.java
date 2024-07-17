package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class ApriltagVisionSubsystem {



    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor myAprilTagProcessor;

    VisionPortal myVisionPortal;

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    public ApriltagVisionSubsystem(HardwareMap Map) {
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        //Configures Apriltag processor
        AprilTagConfig();

        new VisionPortal.Builder().
                addProcessor(processor)
                .addProcessor(myAprilTagProcessor)
                .setCamera(Map.get(WebcamName.class, "Webcam 1"))
                .build();

        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) myAprilTagProcessor, 0);

    }

    public void ApriltagPeriodic() {
    myAprilTagProcessor.getDetections();
    }

    public void AprilTagConfig() {
        // Create a new AprilTag Processor Builder object.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Optional: set other custom features of the AprilTag Processor (4 are shown here).
        myAprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        myAprilTagProcessorBuilder.setDrawAxes(true);        // Default: false.
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);        // Default: false.

// Create an AprilTagProcessor by calling build()
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    }


}
