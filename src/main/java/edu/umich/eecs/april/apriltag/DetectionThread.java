package edu.umich.eecs.april.apriltag;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;
import android.graphics.PorterDuff;
import android.graphics.SurfaceTexture;
import android.graphics.DashPathEffect;
import android.hardware.Camera;
import android.util.Log;
import android.view.TextureView;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class DetectionThread extends Thread {

    private static final String TAG = "DetectionThread";
    private TextureView mTextureView;

    private final TextView mFpsTextView;
    private final TextView mPoseTextView;
    private long mLastFPSRender = System.currentTimeMillis();
    private Camera.Size mCameraSize;
    private static final int MAX_FRAME_QUEUE_SIZE = 1;

    private BlockingQueue<byte[]> mCameraFrameQueue = new LinkedBlockingQueue<>();
    private long mLastEnqueueFrameTime;
    private int mFrameCount = 0;
    private long mLastDetectLatency = 0;

    private float mFovH = 60.0f;
    private float mFovV = 45.0f;

    private volatile boolean m3dMode = false;
    private volatile float mVirtualYaw = -0.5f; // approx -30 degrees
    private volatile float mVirtualPitch = 0.4f; // approx 23 degrees
    private float mFocalX = 0.0f;
    private float mFocalY = 0.0f;
    private float mFocalZ = 1.5f;
    private float mVirtualDistance = 2.0f;

    public void set3DMode(boolean active) {
        m3dMode = active;
    }

    public void updateVirtualCameraOrbit(float dYaw, float dPitch) {
        mVirtualYaw += dYaw;
        mVirtualPitch += dPitch;

        // Clip pitch to prevent flipping upside down
        float maxPitch = (float) Math.toRadians(85);
        float minPitch = (float) Math.toRadians(-85);
        if (mVirtualPitch > maxPitch) mVirtualPitch = maxPitch;
        if (mVirtualPitch < minPitch) mVirtualPitch = minPitch;
    }

    public void setCameraFov(float fovH, float fovV) {
        if (fovH > 0 && fovV > 0) {
            mFovH = fovH;
            mFovV = fovV;
        }
    }

    private static class Pose3D {
        public int id;
        public double tx, ty, tz;
        public double roll, pitch, yaw;
        public double distance;
        public double[] r1, r2, r3, t;
    }

    public static double[] solve8x8(double[][] A, double[] B) {
        int n = 8;
        for (int i = 0; i < n; i++) {
            // Find pivot
            int max = i;
            for (int j = i + 1; j < n; j++) {
                if (Math.abs(A[j][i]) > Math.abs(A[max][i])) {
                    max = j;
                }
            }
            // Swap rows
            double[] tempA = A[i]; A[i] = A[max]; A[max] = tempA;
            double tempB = B[i]; B[i] = B[max]; B[max] = tempB;

            // Singular matrix check
            if (Math.abs(A[i][i]) < 1e-10) {
                return null;
            }

            // Pivot within A and B
            for (int j = i + 1; j < n; j++) {
                double factor = A[j][i] / A[i][i];
                B[j] -= factor * B[i];
                for (int k = i; k < n; k++) {
                    A[j][k] -= factor * A[i][k];
                }
            }
        }

        // Back substitution
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (int j = i + 1; j < n; j++) {
                sum += A[i][j] * x[j];
            }
            x[i] = (B[i] - sum) / A[i][i];
        }
        return x;
    }

    private Pose3D estimatePose(ApriltagDetection detection, double tagSize, double fx, double fy, double cx, double cy) {
        double[] points = detection.p;
        if (points == null || points.length != 8) return null;

        // Destination points normalized
        double[] xNorm = new double[4];
        double[] yNorm = new double[4];
        for (int i = 0; i < 4; i++) {
            xNorm[i] = (points[i * 2] - cx) / fx;
            yNorm[i] = (points[i * 2 + 1] - cy) / fy;
        }

        // Source points on the tag plane
        double s2 = tagSize / 2.0;
        double[] xSrc = new double[]{-s2, s2, s2, -s2};
        double[] ySrc = new double[]{s2, s2, -s2, -s2}; // winding counter-clockwise

        // A h = B
        double[][] A = new double[8][8];
        double[] B = new double[8];
        for (int i = 0; i < 4; i++) {
            int r = i * 2;
            A[r][0] = xSrc[i];
            A[r][1] = ySrc[i];
            A[r][2] = 1.0;
            A[r][3] = 0.0;
            A[r][4] = 0.0;
            A[r][5] = 0.0;
            A[r][6] = -xNorm[i] * xSrc[i];
            A[r][7] = -xNorm[i] * ySrc[i];
            B[r] = xNorm[i];

            A[r + 1][0] = 0.0;
            A[r + 1][1] = 0.0;
            A[r + 1][2] = 0.0;
            A[r + 1][3] = xSrc[i];
            A[r + 1][4] = ySrc[i];
            A[r + 1][5] = 1.0;
            A[r + 1][6] = -yNorm[i] * xSrc[i];
            A[r + 1][7] = -yNorm[i] * ySrc[i];
            B[r + 1] = yNorm[i];
        }

        double[] h = solve8x8(A, B);
        if (h == null) return null;

        // Columns of homography
        double[] h1 = new double[]{h[0], h[3], h[6]};
        double[] h2 = new double[]{h[1], h[4], h[7]};
        double[] h3 = new double[]{h[2], h[5], 1.0};

        double norm1 = Math.sqrt(h1[0]*h1[0] + h1[1]*h1[1] + h1[2]*h1[2]);
        double norm2 = Math.sqrt(h2[0]*h2[0] + h2[1]*h2[1] + h2[2]*h2[2]);
        double lambda = 2.0 / (norm1 + norm2);
        if (lambda < 0) lambda = -lambda;

        // Rotation & translation vectors
        double[] r1 = new double[]{lambda * h1[0], lambda * h1[1], lambda * h1[2]};
        double[] r2 = new double[]{lambda * h2[0], lambda * h2[1], lambda * h2[2]};
        double[] t = new double[]{lambda * h3[0], lambda * h3[1], lambda * h3[2]};

        // Orthogonalize r1 and r2
        double mag1 = Math.sqrt(r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2]);
        if (mag1 < 1e-6) return null;
        r1[0] /= mag1; r1[1] /= mag1; r1[2] /= mag1;

        double dot = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];
        r2[0] -= dot * r1[0]; r2[1] -= dot * r1[1]; r2[2] -= dot * r1[2];

        double mag2 = Math.sqrt(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);
        if (mag2 < 1e-6) return null;
        r2[0] /= mag2; r2[1] /= mag2; r2[2] /= mag2;

        double[] r3 = new double[3];
        r3[0] = r1[1]*r2[2] - r1[2]*r2[1];
        r3[1] = r1[2]*r2[0] - r1[0]*r2[2];
        r3[2] = r1[0]*r2[1] - r1[1]*r2[0];

        // Extract Euler Angles (Roll, Pitch, Yaw)
        double pitch = Math.asin(-r1[2]);
        double roll, yaw;
        if (Math.cos(pitch) > 1e-4) {
            roll = Math.atan2(r2[2], r3[2]);
            yaw = Math.atan2(r1[1], r1[0]);
        } else {
            roll = 0.0;
            yaw = Math.atan2(-r2[0], r2[1]);
        }

        Pose3D pose = new Pose3D();
        pose.id = detection.id;
        pose.tx = t[0];
        pose.ty = t[1];
        pose.tz = t[2];
        pose.roll = Math.toDegrees(roll);
        pose.pitch = Math.toDegrees(pitch);
        pose.yaw = Math.toDegrees(yaw);
        pose.distance = Math.sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        pose.r1 = r1;
        pose.r2 = r2;
        pose.r3 = r3;
        pose.t = t;

        return pose;
    }

    private float[] projectPhysical3DToVirtual(double x, double y, double z, int width, int height) {
        // Translate relative to focal point
        double x1 = x - mFocalX;
        double y1 = y - mFocalY;
        double z1 = z - mFocalZ;

        float yaw = mVirtualYaw;
        float pitch = mVirtualPitch;

        // Rotate around Y axis (Yaw)
        double cosY = Math.cos(yaw);
        double sinY = Math.sin(yaw);
        double x2 = x1 * cosY - z1 * sinY;
        double z2 = x1 * sinY + z1 * cosY;
        double y2 = y1;

        // Rotate around X axis (Pitch)
        double cosP = Math.cos(pitch);
        double sinP = Math.sin(pitch);
        double y3 = y2 * cosP + z2 * sinP;
        double z3 = -y2 * sinP + z2 * cosP;
        double x3 = x2;

        // Apply camera distance along the virtual Z axis
        double zVirtual = z3 + mVirtualDistance;
        double xVirtual = x3;
        double yVirtual = y3;

        if (zVirtual <= 0.1) {
            return null; // Clip behind camera
        }

        double fv = 0.8 * Math.min(width, height);

        float px = (float) (xVirtual * fv / zVirtual + width / 2.0);
        float py = (float) (yVirtual * fv / zVirtual + height / 2.0);

        return new float[]{px, py};
    }

    private float[] projectTagSpacePointToVirtual(double xTag, double yTag, double zTag, Pose3D pose, int width, int height) {
        double xCam = pose.r1[0] * xTag + pose.r2[0] * yTag + pose.r3[0] * zTag + pose.t[0];
        double yCam = pose.r1[1] * xTag + pose.r2[1] * yTag + pose.r3[1] * zTag + pose.t[1];
        double zCam = pose.r1[2] * xTag + pose.r2[2] * yTag + pose.r3[2] * zTag + pose.t[2];
        return projectPhysical3DToVirtual(xCam, yCam, zCam, width, height);
    }

    private float[] project3DPoint(double xTag, double yTag, double zTag,
                                   double[] r1, double[] r2, double[] r3, double[] t,
                                   double fx, double fy, double cx, double cy,
                                   float scaleDetectionX, float scaleDetectionY, Canvas canvas) {
        double xCam = r1[0] * xTag + r2[0] * yTag + r3[0] * zTag + t[0];
        double yCam = r1[1] * xTag + r2[1] * yTag + r3[1] * zTag + t[1];
        double zCam = r1[2] * xTag + r2[2] * yTag + r3[2] * zTag + t[2];

        if (zCam <= 1e-3) {
            return null; // Behind camera
        }

        double xPix = xCam * (fx / zCam) + cx;
        double yPix = yCam * (fy / zCam) + cy;

        float xCanvas = (float) (canvas.getWidth() - yPix * scaleDetectionY);
        float yCanvas = (float) (xPix * scaleDetectionX);

        return new float[]{xCanvas, yCanvas};
    }

    public DetectionThread(TextureView textureView, TextView fpsTextView, TextView poseTextView) {
        mTextureView = textureView;
        mFpsTextView = fpsTextView;
        mPoseTextView = poseTextView;
        mTextureView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
                // Do nothing
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
                // Do nothing
            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {
                // Do nothing
            }
        });
    }

    public void destroy() {
        mCameraFrameQueue.clear();
        mCameraFrameQueue = null;
    }

    public void enqueueCameraFrame(byte[] data, Camera.Size cameraSize) throws InterruptedException {
        if (mCameraSize == null || mCameraSize.width != cameraSize.width || mCameraSize.height != cameraSize.height) {
            mCameraFrameQueue.clear();
            mCameraSize = cameraSize;
            Log.w(TAG, "Camera size changed during preview");
        }

        if (mCameraFrameQueue == null) {
            Log.w(TAG, "Camera frame queue is null, skipping frame");
            return;
        }

        if (mCameraFrameQueue.size() == MAX_FRAME_QUEUE_SIZE) {
            mCameraFrameQueue.clear();
            Log.w(TAG, "Camera frame queue is full, clearing buffer");
        }

        mCameraFrameQueue.put(data);
        mLastEnqueueFrameTime = System.currentTimeMillis();

        Log.i(TAG, "Buffer length: " + mCameraFrameQueue.size());
    }

    private void updateFps() {
        long now = System.currentTimeMillis();
        long diff = now - mLastFPSRender;
        mFrameCount++;
        if (diff >= 1000) {
            final double fps = 1000.0 / diff * mFrameCount;
            mFpsTextView.post(new Runnable() {
                @Override
                public void run() {
                    mFpsTextView.setText(String.format("%.2f fps Detect+Render\n%d ms Detect+Render Latency", fps, mLastDetectLatency));
                }
            });
            mLastFPSRender = now;
            mFrameCount = 0;
        }
    }

    private ArrayList<ApriltagDetection> processCameraFrame(byte[] data, Camera.Size cameraSize)  {
        try {
            return ApriltagNative.apriltag_detect_yuv(data, cameraSize.width, cameraSize.height);
        } catch (Exception e) {
            Log.e(TAG, "Unhandled exception when detecting tags: " + e);
            return new ArrayList<>();
        }
    }

    private Pose3D renderDetection(ApriltagDetection detection, Canvas canvas) {
        if (mCameraSize == null) return null;

        Paint fillPaint = new Paint();
        fillPaint.setColor(0xFF39FF14); // Neon green
        fillPaint.setAlpha(40);          // Light semi-transparent fill
        fillPaint.setStyle(Paint.Style.FILL);
        fillPaint.setAntiAlias(true);

        Paint borderPaint = new Paint();
        borderPaint.setColor(0xFF39FF14); // Neon green
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(8);
        borderPaint.setAntiAlias(true);

        float scaleDetectionX = (float)(canvas.getHeight()) / mCameraSize.width; // Converts detection x to render y
        float scaleDetectionY = (float)(canvas.getWidth()) / mCameraSize.height; // Converts detection y to render x (still needs offset)

        double[] points = detection.p;
        if (points == null || points.length != 8) {
            Log.w(TAG, "invalid detection coordinates");
            return null;
        }

        // Convert detection points to canvas points
        float[] xPointsCanvas = new float[4];
        float[] yPointsCanvas = new float[4];
        for (int i = 0; i < 4; i++) {
            xPointsCanvas[i] = (float) (canvas.getWidth() - points[i * 2 + 1] * scaleDetectionY);
            yPointsCanvas[i] = (float) (points[i * 2] * scaleDetectionX);
        }

        // Render filled outline of detections
        Path fillPath = new Path();
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                fillPath.moveTo(xPointsCanvas[i], yPointsCanvas[i]);
            } else {
                fillPath.lineTo(xPointsCanvas[i], yPointsCanvas[i]);
            }
        }
        fillPath.close();
        canvas.drawPath(fillPath, fillPaint);

        // Render stroke outline of detections (uniform neon green border)
        canvas.drawPath(fillPath, borderPaint);

        // Retrieve AprilTag Size from settings
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mTextureView.getContext());
        String sizeStr = sharedPreferences.getString("apriltag_size", "0.165");
        double tagSize = 0.165;
        try {
            tagSize = Double.parseDouble(sizeStr);
        } catch (NumberFormatException e) {
            Log.e(TAG, "Invalid apriltag_size setting, defaulting to 0.165");
        }

        // Check for manual camera calibration settings override
        boolean calOverride = sharedPreferences.getBoolean("calibration_override", false);
        double cx = mCameraSize.width / 2.0;
        double cy = mCameraSize.height / 2.0;
        double fx, fy;

        if (calOverride) {
            double customFx = 600.0;
            double customFy = 600.0;
            double customCx = 0.0;
            double customCy = 0.0;
            try {
                customFx = Double.parseDouble(sharedPreferences.getString("calibration_fx", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fx, using default 600.0");
            }
            try {
                customFy = Double.parseDouble(sharedPreferences.getString("calibration_fy", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fy, using default 600.0");
            }
            try {
                customCx = Double.parseDouble(sharedPreferences.getString("calibration_cx", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cx, using default 0.0");
            }
            try {
                customCy = Double.parseDouble(sharedPreferences.getString("calibration_cy", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cy, using default 0.0");
            }

            fx = customFx;
            fy = customFy;
            if (customCx > 0.0) {
                cx = customCx;
            }
            if (customCy > 0.0) {
                cy = customCy;
            }
        } else {
            // Calculate Intrinsics from dynamic view angles
            fx = cx / Math.tan(Math.toRadians(mFovH / 2.0));
            fy = cy / Math.tan(Math.toRadians(mFovV / 2.0));
        }

        // Estimate 3D Pose
        Pose3D pose = estimatePose(detection, tagSize, fx, fy, cx, cy);

        // Render 3D overlays if pose estimation succeeded
        if (pose != null) {
            double s2 = tagSize / 2.0;

            // Project 8 corners of the 3D box
            // Back face (on the tag): z = 0
            float[] p0 = project3DPoint(-s2, s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p1 = project3DPoint(s2, s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p2 = project3DPoint(s2, -s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p3 = project3DPoint(-s2, -s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);

            // Front face (projected out towards the camera): z = -tagSize
            float[] p4 = project3DPoint(-s2, s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p5 = project3DPoint(s2, s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p6 = project3DPoint(s2, -s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] p7 = project3DPoint(-s2, -s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);

            if (p0 != null && p1 != null && p2 != null && p3 != null &&
                p4 != null && p5 != null && p6 != null && p7 != null) {

                // Draw faces of the 3D box with light transparent fill
                Paint facePaint = new Paint();
                facePaint.setColor(0x1F00E5FF); // Transparent neon cyan
                facePaint.setStyle(Paint.Style.FILL);
                facePaint.setAntiAlias(true);

                // Front face
                Path frontPath = new Path();
                frontPath.moveTo(p4[0], p4[1]);
                frontPath.lineTo(p5[0], p5[1]);
                frontPath.lineTo(p6[0], p6[1]);
                frontPath.lineTo(p7[0], p7[1]);
                frontPath.close();
                canvas.drawPath(frontPath, facePaint);

                // Left face
                Path leftPath = new Path();
                leftPath.moveTo(p0[0], p0[1]);
                leftPath.lineTo(p3[0], p3[1]);
                leftPath.lineTo(p7[0], p7[1]);
                leftPath.lineTo(p4[0], p4[1]);
                leftPath.close();
                canvas.drawPath(leftPath, facePaint);

                // Right face
                Path rightPath = new Path();
                rightPath.moveTo(p1[0], p1[1]);
                rightPath.lineTo(p2[0], p2[1]);
                rightPath.lineTo(p6[0], p6[1]);
                rightPath.lineTo(p5[0], p5[1]);
                rightPath.close();
                canvas.drawPath(rightPath, facePaint);

                // Top face
                Path topPath = new Path();
                topPath.moveTo(p0[0], p0[1]);
                topPath.lineTo(p1[0], p1[1]);
                topPath.lineTo(p5[0], p5[1]);
                topPath.lineTo(p4[0], p4[1]);
                topPath.close();
                canvas.drawPath(topPath, facePaint);

                // Bottom face
                Path bottomPath = new Path();
                bottomPath.moveTo(p3[0], p3[1]);
                bottomPath.lineTo(p2[0], p2[1]);
                bottomPath.lineTo(p6[0], p6[1]);
                bottomPath.lineTo(p7[0], p7[1]);
                bottomPath.close();
                canvas.drawPath(bottomPath, facePaint);

                // Draw wireframe outlines (edges)
                Paint edgePaint = new Paint();
                edgePaint.setColor(0xFF00E5FF); // Neon cyan
                edgePaint.setStyle(Paint.Style.STROKE);
                edgePaint.setStrokeWidth(4);
                edgePaint.setAntiAlias(true);

                // Draw back face outline
                canvas.drawLine(p0[0], p0[1], p1[0], p1[1], edgePaint);
                canvas.drawLine(p1[0], p1[1], p2[0], p2[1], edgePaint);
                canvas.drawLine(p2[0], p2[1], p3[0], p3[1], edgePaint);
                canvas.drawLine(p3[0], p3[1], p0[0], p0[1], edgePaint);

                // Draw front face outline
                canvas.drawLine(p4[0], p4[1], p5[0], p5[1], edgePaint);
                canvas.drawLine(p5[0], p5[1], p6[0], p6[1], edgePaint);
                canvas.drawLine(p6[0], p6[1], p7[0], p7[1], edgePaint);
                canvas.drawLine(p7[0], p7[1], p4[0], p4[1], edgePaint);

                // Draw connecting edges
                canvas.drawLine(p0[0], p0[1], p4[0], p4[1], edgePaint);
                canvas.drawLine(p1[0], p1[1], p5[0], p5[1], edgePaint);
                canvas.drawLine(p2[0], p2[1], p6[0], p6[1], edgePaint);
                canvas.drawLine(p3[0], p3[1], p7[0], p7[1], edgePaint);
            }

            // Draw 3D coordinate axes
            float[] origin = project3DPoint(0.0, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] xAxis = project3DPoint(tagSize, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] yAxis = project3DPoint(0.0, tagSize, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] zAxis = project3DPoint(0.0, 0.0, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);

            if (origin != null) {
                Paint axisPaint = new Paint();
                axisPaint.setStyle(Paint.Style.STROKE);
                axisPaint.setStrokeWidth(6);
                axisPaint.setAntiAlias(true);

                // X-axis: Red
                if (xAxis != null) {
                    axisPaint.setColor(0xFFFF3F3F);
                    canvas.drawLine(origin[0], origin[1], xAxis[0], xAxis[1], axisPaint);
                }
                // Y-axis: Green
                if (yAxis != null) {
                    axisPaint.setColor(0xFF3FDF3F);
                    canvas.drawLine(origin[0], origin[1], yAxis[0], yAxis[1], axisPaint);
                }
                // Z-axis: Blue
                if (zAxis != null) {
                    axisPaint.setColor(0xFF3F3FFF);
                    canvas.drawLine(origin[0], origin[1], zAxis[0], zAxis[1], axisPaint);
                }
            }
        }

        // Render target-center indicator crosshair (circle + cross)
        float tagCenterX = (float) (canvas.getWidth() - detection.c[1] * scaleDetectionY);
        float tagCenterY = (float) (detection.c[0] * scaleDetectionX);

        Paint targetIndicatorPaint = new Paint();
        targetIndicatorPaint.setColor(0xFF39FF14); // Neon green
        targetIndicatorPaint.setStyle(Paint.Style.STROKE);
        targetIndicatorPaint.setStrokeWidth(4);
        targetIndicatorPaint.setAntiAlias(true);

        canvas.drawCircle(tagCenterX, tagCenterY, 12, targetIndicatorPaint);
        canvas.drawLine(tagCenterX - 18, tagCenterY, tagCenterX + 18, tagCenterY, targetIndicatorPaint);
        canvas.drawLine(tagCenterX, tagCenterY - 18, tagCenterX, tagCenterY + 18, targetIndicatorPaint);

        // Render Tag ID badge
        Paint textPaint = new Paint();
        textPaint.setColor(0xFF39FF14); // Neon green text
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        textPaint.setAntiAlias(true);

        String badgeText = "ID: " + detection.id;
        if (pose != null) {
            badgeText = String.format("ID: %d (%.2fm)", detection.id, pose.distance);
        }
        float textWidth = textPaint.measureText(badgeText);
        Paint.FontMetrics fm = textPaint.getFontMetrics();
        float textHeight = fm.descent - fm.ascent;

        // Find minimum and maximum Y to position the badge
        float targetMinY = yPointsCanvas[0];
        float targetMaxY = yPointsCanvas[0];
        for (int i = 1; i < 4; i++) {
            if (yPointsCanvas[i] < targetMinY) {
                targetMinY = yPointsCanvas[i];
            }
            if (yPointsCanvas[i] > targetMaxY) {
                targetMaxY = yPointsCanvas[i];
            }
        }

        float paddingX = 16;
        float paddingY = 8;
        float margin = 20;

        float badgeBottom = targetMinY - margin;
        if (badgeBottom - textHeight - 2 * paddingY < 10) {
            badgeBottom = targetMaxY + margin + textHeight + 2 * paddingY;
        }

        float badgeTop = badgeBottom - textHeight - 2 * paddingY;
        float badgeLeft = tagCenterX - textWidth / 2f - paddingX;
        float badgeRight = tagCenterX + textWidth / 2f + paddingX;

        Paint bgPaint = new Paint();
        bgPaint.setColor(0xFF00E5FF); // Neon cyan background border for 3D estimation
        bgPaint.setColor(0xFF000000); // Black
        bgPaint.setAlpha(180);         // Semi-transparent
        bgPaint.setStyle(Paint.Style.FILL);
        bgPaint.setAntiAlias(true);

        canvas.drawRoundRect(badgeLeft, badgeTop, badgeRight, badgeBottom, 12, 12, bgPaint);

        // Draw tag ID text centered
        float textX = tagCenterX - textWidth / 2f;
        float textY = badgeBottom - paddingY - fm.descent;
        canvas.drawText(badgeText, textX, textY, textPaint);

        return pose;
    }

    private void render3DView(ArrayList<ApriltagDetection> detections, Canvas canvas) {
        canvas.drawColor(0xFF1E1E1E); // RViz dark grey

        int width = canvas.getWidth();
        int height = canvas.getHeight();

        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mTextureView.getContext());
        String sizeStr = sharedPreferences.getString("apriltag_size", "0.165");
        double tagSize = 0.165;
        try {
            tagSize = Double.parseDouble(sizeStr);
        } catch (NumberFormatException e) {
            Log.e(TAG, "Invalid apriltag_size setting, defaulting to 0.165");
        }

        boolean calOverride = sharedPreferences.getBoolean("calibration_override", false);
        double cx = mCameraSize != null ? mCameraSize.width / 2.0 : width / 2.0;
        double cy = mCameraSize != null ? mCameraSize.height / 2.0 : height / 2.0;
        double fx, fy;

        if (calOverride) {
            double customFx = 600.0;
            double customFy = 600.0;
            double customCx = 0.0;
            double customCy = 0.0;
            try {
                customFx = Double.parseDouble(sharedPreferences.getString("calibration_fx", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fx, using default 600.0");
            }
            try {
                customFy = Double.parseDouble(sharedPreferences.getString("calibration_fy", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fy, using default 600.0");
            }
            try {
                customCx = Double.parseDouble(sharedPreferences.getString("calibration_cx", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cx, using default 0.0");
            }
            try {
                customCy = Double.parseDouble(sharedPreferences.getString("calibration_cy", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cy, using default 0.0");
            }

            fx = customFx;
            fy = customFy;
            if (customCx > 0.0) {
                cx = customCx;
            }
            if (customCy > 0.0) {
                cy = customCy;
            }
        } else {
            fx = cx / Math.tan(Math.toRadians(mFovH / 2.0));
            fy = cy / Math.tan(Math.toRadians(mFovV / 2.0));
        }

        double sumX = 0, sumY = 0, sumZ = 0;
        int validCount = 0;

        ArrayList<Pose3D> poses = new ArrayList<>();

        for (ApriltagDetection detection : detections) {
            Pose3D pose = estimatePose(detection, tagSize, fx, fy, cx, cy);
            if (pose != null) {
                poses.add(pose);
                sumX += pose.tx;
                sumY += pose.ty;
                sumZ += pose.tz;
                validCount++;
            }
        }

        double targetFocalX = 0.0;
        double targetFocalY = 0.0;
        double targetFocalZ = 1.5;
        double targetDistance = 2.0;

        if (validCount > 0) {
            targetFocalX = sumX / validCount;
            targetFocalY = sumY / validCount;
            targetFocalZ = sumZ / validCount;
            double avgDistance = Math.sqrt(targetFocalX * targetFocalX + targetFocalY * targetFocalY + targetFocalZ * targetFocalZ);
            targetDistance = Math.max(1.5, avgDistance + 1.0);
        }

        // Smoothly interpolate camera focus point and distance
        mFocalX = 0.9f * mFocalX + 0.1f * (float) targetFocalX;
        mFocalY = 0.9f * mFocalY + 0.1f * (float) targetFocalY;
        mFocalZ = 0.9f * mFocalZ + 0.1f * (float) targetFocalZ;
        mVirtualDistance = 0.9f * mVirtualDistance + 0.1f * (float) targetDistance;

        // 1. Draw floor grid (parallel to X-Z plane, Y = 0.5 meters)
        Paint gridPaint = new Paint();
        gridPaint.setColor(0x3F888888); // Semi-transparent grey
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(2);
        gridPaint.setAntiAlias(true);

        for (float xVal = -2.5f; xVal <= 2.5f; xVal += 0.5f) {
            float[] pStart = projectPhysical3DToVirtual(xVal, 0.5, -1.0, width, height);
            float[] pEnd = projectPhysical3DToVirtual(xVal, 0.5, 6.0, width, height);
            if (pStart != null && pEnd != null) {
                canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], gridPaint);
            }
        }
        for (float zVal = -1.0f; zVal <= 6.0f; zVal += 0.5f) {
            float[] pStart = projectPhysical3DToVirtual(-2.5, 0.5, zVal, width, height);
            float[] pEnd = projectPhysical3DToVirtual(2.5, 0.5, zVal, width, height);
            if (pStart != null && pEnd != null) {
                canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], gridPaint);
            }
        }

        // 2. Draw camera axes and frustum at the origin (0,0,0)
        float[] origin = projectPhysical3DToVirtual(0, 0, 0, width, height);
        float[] xAxisCam = projectPhysical3DToVirtual(0.25, 0, 0, width, height);
        float[] yAxisCam = projectPhysical3DToVirtual(0, 0.25, 0, width, height);
        float[] zAxisCam = projectPhysical3DToVirtual(0, 0, 0.25, width, height);

        Paint axisPaint = new Paint();
        axisPaint.setStyle(Paint.Style.STROKE);
        axisPaint.setStrokeWidth(6);
        axisPaint.setAntiAlias(true);

        if (origin != null) {
            if (xAxisCam != null) {
                axisPaint.setColor(0xFFFF3F3F);
                canvas.drawLine(origin[0], origin[1], xAxisCam[0], xAxisCam[1], axisPaint);
            }
            if (yAxisCam != null) {
                axisPaint.setColor(0xFF3FDF3F);
                canvas.drawLine(origin[0], origin[1], yAxisCam[0], yAxisCam[1], axisPaint);
            }
            if (zAxisCam != null) {
                axisPaint.setColor(0xFF3F3FFF);
                canvas.drawLine(origin[0], origin[1], zAxisCam[0], zAxisCam[1], axisPaint);
            }
        }

        float[] pf0 = projectPhysical3DToVirtual(-0.18, -0.13, 0.35, width, height);
        float[] pf1 = projectPhysical3DToVirtual(0.18, -0.13, 0.35, width, height);
        float[] pf2 = projectPhysical3DToVirtual(0.18, 0.13, 0.35, width, height);
        float[] pf3 = projectPhysical3DToVirtual(-0.18, 0.13, 0.35, width, height);

        if (origin != null && pf0 != null && pf1 != null && pf2 != null && pf3 != null) {
            Paint frustumPaint = new Paint();
            frustumPaint.setColor(0x7FCCCCCC);
            frustumPaint.setStyle(Paint.Style.STROKE);
            frustumPaint.setStrokeWidth(3);
            frustumPaint.setAntiAlias(true);

            canvas.drawLine(origin[0], origin[1], pf0[0], pf0[1], frustumPaint);
            canvas.drawLine(origin[0], origin[1], pf1[0], pf1[1], frustumPaint);
            canvas.drawLine(origin[0], origin[1], pf2[0], pf2[1], frustumPaint);
            canvas.drawLine(origin[0], origin[1], pf3[0], pf3[1], frustumPaint);

            canvas.drawLine(pf0[0], pf0[1], pf1[0], pf1[1], frustumPaint);
            canvas.drawLine(pf1[0], pf1[1], pf2[0], pf2[1], frustumPaint);
            canvas.drawLine(pf2[0], pf2[1], pf3[0], pf3[1], frustumPaint);
            canvas.drawLine(pf3[0], pf3[1], pf0[0], pf0[1], frustumPaint);
        }

        // 3. Draw detected tags (axes, wireframe cubes, tracking rays, labels)
        for (Pose3D pose : poses) {
            float[] tagCenter = projectPhysical3DToVirtual(pose.tx, pose.ty, pose.tz, width, height);
            if (origin != null && tagCenter != null) {
                Paint rayPaint = new Paint();
                rayPaint.setColor(0xFFFFD700); // Golden yellow tracking rays
                rayPaint.setStyle(Paint.Style.STROKE);
                rayPaint.setStrokeWidth(4);
                rayPaint.setAntiAlias(true);
                rayPaint.setPathEffect(new DashPathEffect(new float[]{15, 10}, 0));
                canvas.drawLine(origin[0], origin[1], tagCenter[0], tagCenter[1], rayPaint);
            }

            double L = tagSize;
            float[] pxAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r1[0], pose.ty + L * pose.r1[1], pose.tz + L * pose.r1[2], width, height);
            float[] pyAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r2[0], pose.ty + L * pose.r2[1], pose.tz + L * pose.r2[2], width, height);
            float[] pzAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r3[0], pose.ty + L * pose.r3[1], pose.tz + L * pose.r3[2], width, height);

            if (tagCenter != null) {
                Paint tagAxisPaint = new Paint();
                tagAxisPaint.setStyle(Paint.Style.STROKE);
                tagAxisPaint.setStrokeWidth(6);
                tagAxisPaint.setAntiAlias(true);

                if (pxAxis != null) {
                    tagAxisPaint.setColor(0xFFFF3F3F);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pxAxis[0], pxAxis[1], tagAxisPaint);
                }
                if (pyAxis != null) {
                    tagAxisPaint.setColor(0xFF3FDF3F);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pyAxis[0], pyAxis[1], tagAxisPaint);
                }
                if (pzAxis != null) {
                    tagAxisPaint.setColor(0xFF3F3FFF);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pzAxis[0], pzAxis[1], tagAxisPaint);
                }
            }

            double s2 = tagSize / 2.0;
            float[] c0 = projectTagSpacePointToVirtual(-s2, s2, 0.0, pose, width, height);
            float[] c1 = projectTagSpacePointToVirtual(s2, s2, 0.0, pose, width, height);
            float[] c2 = projectTagSpacePointToVirtual(s2, -s2, 0.0, pose, width, height);
            float[] c3 = projectTagSpacePointToVirtual(-s2, -s2, 0.0, pose, width, height);
            float[] c4 = projectTagSpacePointToVirtual(-s2, s2, -tagSize, pose, width, height);
            float[] c5 = projectTagSpacePointToVirtual(s2, s2, -tagSize, pose, width, height);
            float[] c6 = projectTagSpacePointToVirtual(s2, -s2, -tagSize, pose, width, height);
            float[] c7 = projectTagSpacePointToVirtual(-s2, -s2, -tagSize, pose, width, height);

            if (c0 != null && c1 != null && c2 != null && c3 != null &&
                c4 != null && c5 != null && c6 != null && c7 != null) {

                Paint facePaint = new Paint();
                facePaint.setColor(0x1F00E5FF);
                facePaint.setStyle(Paint.Style.FILL);
                facePaint.setAntiAlias(true);

                Path fPath = new Path();
                fPath.moveTo(c4[0], c4[1]);
                fPath.lineTo(c5[0], c5[1]);
                fPath.lineTo(c6[0], c6[1]);
                fPath.lineTo(c7[0], c7[1]);
                fPath.close();
                canvas.drawPath(fPath, facePaint);

                Path lPath = new Path();
                lPath.moveTo(c0[0], c0[1]);
                lPath.lineTo(c3[0], c3[1]);
                lPath.lineTo(c7[0], c7[1]);
                lPath.lineTo(c4[0], c4[1]);
                lPath.close();
                canvas.drawPath(lPath, facePaint);

                Path rPath = new Path();
                rPath.moveTo(c1[0], c1[1]);
                rPath.lineTo(c2[0], c2[1]);
                rPath.lineTo(c6[0], c6[1]);
                rPath.lineTo(c5[0], c5[1]);
                rPath.close();
                canvas.drawPath(rPath, facePaint);

                Path tPath = new Path();
                tPath.moveTo(c0[0], c0[1]);
                tPath.lineTo(c1[0], c1[1]);
                tPath.lineTo(c5[0], c5[1]);
                tPath.lineTo(c4[0], c4[1]);
                tPath.close();
                canvas.drawPath(tPath, facePaint);

                Path bPath = new Path();
                bPath.moveTo(c3[0], c3[1]);
                bPath.lineTo(c2[0], c2[1]);
                bPath.lineTo(c6[0], c6[1]);
                bPath.lineTo(c7[0], c7[1]);
                bPath.close();
                canvas.drawPath(bPath, facePaint);

                Paint edgePaint = new Paint();
                edgePaint.setColor(0xFF00E5FF);
                edgePaint.setStyle(Paint.Style.STROKE);
                edgePaint.setStrokeWidth(4);
                edgePaint.setAntiAlias(true);

                canvas.drawLine(c0[0], c0[1], c1[0], c1[1], edgePaint);
                canvas.drawLine(c1[0], c1[1], c2[0], c2[1], edgePaint);
                canvas.drawLine(c2[0], c2[1], c3[0], c3[1], edgePaint);
                canvas.drawLine(c3[0], c3[1], c0[0], c0[1], edgePaint);

                canvas.drawLine(c4[0], c4[1], c5[0], c5[1], edgePaint);
                canvas.drawLine(c5[0], c5[1], c6[0], c6[1], edgePaint);
                canvas.drawLine(c6[0], c6[1], c7[0], c7[1], edgePaint);
                canvas.drawLine(c7[0], c7[1], c4[0], c4[1], edgePaint);

                canvas.drawLine(c0[0], c0[1], c4[0], c4[1], edgePaint);
                canvas.drawLine(c1[0], c1[1], c5[0], c5[1], edgePaint);
                canvas.drawLine(c2[0], c2[1], c6[0], c6[1], edgePaint);
                canvas.drawLine(c3[0], c3[1], c7[0], c7[1], edgePaint);
            }

            float[] badgePos = projectPhysical3DToVirtual(pose.tx, pose.ty - 0.15, pose.tz, width, height);
            if (badgePos != null) {
                Paint textPaint = new Paint();
                textPaint.setColor(0xFF39FF14);
                textPaint.setTextSize(36);
                textPaint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
                textPaint.setAntiAlias(true);

                String badgeText = String.format("ID: %d (%.2fm)", pose.id, pose.distance);
                float textWidth = textPaint.measureText(badgeText);
                Paint.FontMetrics fm = textPaint.getFontMetrics();
                float textHeight = fm.descent - fm.ascent;

                float paddingX = 12;
                float paddingY = 6;
                float badgeLeft = badgePos[0] - textWidth / 2f - paddingX;
                float badgeRight = badgePos[0] + textWidth / 2f + paddingX;
                float badgeTop = badgePos[1] - textHeight / 2f - paddingY;
                float badgeBottom = badgePos[1] + textHeight / 2f + paddingY;

                Paint bgPaint = new Paint();
                bgPaint.setColor(0xFF000000);
                bgPaint.setAlpha(180);
                bgPaint.setStyle(Paint.Style.FILL);
                bgPaint.setAntiAlias(true);

                canvas.drawRoundRect(badgeLeft, badgeTop, badgeRight, badgeBottom, 8, 8, bgPaint);

                float textX = badgePos[0] - textWidth / 2f;
                float textY = badgeBottom - paddingY - fm.descent;
                canvas.drawText(badgeText, textX, textY, textPaint);
            }
        }

        // 4. Update diagnostics card telemetry
        Pose3D closestPose = null;
        double minDistance = Double.MAX_VALUE;
        for (Pose3D p : poses) {
            if (p.distance < minDistance) {
                minDistance = p.distance;
                closestPose = p;
            }
        }

        if (mPoseTextView != null) {
            if (closestPose != null) {
                final String telemetry = String.format("Target ID: %d\n" +
                                "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                        closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                        closestPose.roll, closestPose.pitch, closestPose.yaw);
                mPoseTextView.post(new Runnable() {
                    @Override
                    public void run() {
                        mPoseTextView.setText(telemetry);
                    }
                });
            } else {
                mPoseTextView.post(new Runnable() {
                    @Override
                    public void run() {
                        mPoseTextView.setText("No Target");
                    }
                });
            }
        }
    }

    private void renderDetections(ArrayList<ApriltagDetection> detections) {
        Canvas canvas = mTextureView.lockCanvas();
        if (canvas == null) return;
        try {
            if (m3dMode) {
                render3DView(detections, canvas);
                return;
            }

            canvas.drawColor(0, PorterDuff.Mode.CLEAR);

            ApriltagDetection closestDetection = null;
            double minDistance = Double.MAX_VALUE;
            float closestTagX = 0;
            float closestTagY = 0;
            Pose3D closestPose = null;

            float screenCenterX = canvas.getWidth() / 2f;
            float screenCenterY = canvas.getHeight() / 2f;

            float scaleDetectionX = 0;
            float scaleDetectionY = 0;
            if (mCameraSize != null) {
                scaleDetectionX = (float)(canvas.getHeight()) / mCameraSize.width;
                scaleDetectionY = (float)(canvas.getWidth()) / mCameraSize.height;
            }

            for (ApriltagDetection detection : detections) {
                Pose3D pose = renderDetection(detection, canvas);

                if (mCameraSize != null) {
                    float tagCenterX = (float) (canvas.getWidth() - detection.c[1] * scaleDetectionY);
                    float tagCenterY = (float) (detection.c[0] * scaleDetectionX);
                    double dist = Math.hypot(tagCenterX - screenCenterX, tagCenterY - screenCenterY);
                    if (dist < minDistance) {
                        minDistance = dist;
                        closestDetection = detection;
                        closestTagX = tagCenterX;
                        closestTagY = tagCenterY;
                        closestPose = pose;
                    }
                }
            }

            // Draw lock-on tracking line to the closest target
            if (closestDetection != null) {
                Paint linePaint = new Paint();
                linePaint.setColor(0xFF00E5FF); // Neon cyan
                linePaint.setStyle(Paint.Style.STROKE);
                linePaint.setStrokeWidth(6);
                linePaint.setAntiAlias(true);
                linePaint.setPathEffect(new DashPathEffect(new float[]{15, 10}, 0));
                canvas.drawLine(screenCenterX, screenCenterY, closestTagX, closestTagY, linePaint);
            }

            // Draw central target reticle (neon cyan)
            Paint reticlePaint = new Paint();
            reticlePaint.setColor(0xFF00E5FF); // Neon cyan
            reticlePaint.setStyle(Paint.Style.STROKE);
            reticlePaint.setStrokeWidth(4);
            reticlePaint.setAntiAlias(true);

            // Center dot
            Paint dotPaint = new Paint();
            dotPaint.setColor(0xFF00E5FF);
            dotPaint.setStyle(Paint.Style.FILL);
            dotPaint.setAntiAlias(true);
            canvas.drawCircle(screenCenterX, screenCenterY, 4, dotPaint);

            // Center inner/outer circles and crosshair ticks
            canvas.drawCircle(screenCenterX, screenCenterY, 30, reticlePaint);
            canvas.drawLine(screenCenterX - 45, screenCenterY, screenCenterX - 15, screenCenterY, reticlePaint);
            canvas.drawLine(screenCenterX + 15, screenCenterY, screenCenterX + 45, screenCenterY, reticlePaint);
            canvas.drawLine(screenCenterX, screenCenterY - 45, screenCenterX, screenCenterY - 15, reticlePaint);
            canvas.drawLine(screenCenterX, screenCenterY + 15, screenCenterX, screenCenterY + 45, reticlePaint);

            // Update pose telemetry
            if (mPoseTextView != null) {
                if (closestPose != null) {
                    final String telemetry = String.format("Target ID: %d\n" +
                                    "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                    "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                            closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                            closestPose.roll, closestPose.pitch, closestPose.yaw);
                    mPoseTextView.post(new Runnable() {
                        @Override
                        public void run() {
                            mPoseTextView.setText(telemetry);
                        }
                    });
                } else {
                    mPoseTextView.post(new Runnable() {
                        @Override
                        public void run() {
                            mPoseTextView.setText("No Target");
                        }
                    });
                }
            }

        } catch (Exception e) {
            Log.e(TAG, "Error rendering detections: " + e.getMessage());
        } finally {
            mTextureView.unlockCanvasAndPost(canvas);
        }
    }

    public void initialize() {
        Log.i(TAG, "Detection thread initialize");
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            updateFps();

            if (mCameraFrameQueue == null) {
                continue;
            }

            byte[] data;
            try {
                data = mCameraFrameQueue.take();
            } catch (InterruptedException e) {
                Log.i(TAG, "Interrupted while waiting for camera frame: " + e.getMessage());
                break;
            }

            ArrayList<ApriltagDetection> detections = processCameraFrame(data, mCameraSize);
            renderDetections(detections);

            mLastDetectLatency = (System.currentTimeMillis() - mLastEnqueueFrameTime);
        }
    }
}