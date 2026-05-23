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

    public interface BufferReleaseListener {
        void onBufferReleased(byte[] data);
    }

    private volatile BufferReleaseListener mBufferReleaseListener;

    public void setBufferReleaseListener(BufferReleaseListener listener) {
        mBufferReleaseListener = listener;
    }

    // Reusable drawing paints to avoid allocations inside rendering loops
    private final Paint mFillPaint = new Paint();
    private final Paint mBorderPaint = new Paint();
    private final Paint mFacePaint = new Paint();
    private final Paint mEdgePaint = new Paint();
    private final Paint mAxisPaint = new Paint();
    private final Paint mTargetIndicatorPaint = new Paint();
    private final Paint mTextPaint = new Paint();
    private final Paint mBgPaint = new Paint();
    private final Paint mGridPaint = new Paint();
    private final Paint mFrustumPaint = new Paint();
    private final Paint mRayPaint = new Paint();
    private final Paint mTagAxisPaint = new Paint();
    private final Paint mReticlePaint = new Paint();
    private final Paint mDotPaint = new Paint();
    private final Paint mLinePaint = new Paint();

    // Reusable path to avoid allocations inside rendering loops
    private final Path mPath = new Path();

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

        // Initialize reusable Paints
        mFillPaint.setColor(0xFF39FF14); // Neon green
        mFillPaint.setAlpha(40);          // Light semi-transparent fill
        mFillPaint.setStyle(Paint.Style.FILL);
        mFillPaint.setAntiAlias(true);

        mBorderPaint.setColor(0xFF39FF14); // Neon green
        mBorderPaint.setStyle(Paint.Style.STROKE);
        mBorderPaint.setStrokeWidth(8);
        mBorderPaint.setAntiAlias(true);

        mFacePaint.setColor(0x1F00E5FF); // Transparent neon cyan
        mFacePaint.setStyle(Paint.Style.FILL);
        mFacePaint.setAntiAlias(true);

        mEdgePaint.setColor(0xFF00E5FF); // Neon cyan
        mEdgePaint.setStyle(Paint.Style.STROKE);
        mEdgePaint.setStrokeWidth(4);
        mEdgePaint.setAntiAlias(true);

        mAxisPaint.setStyle(Paint.Style.STROKE);
        mAxisPaint.setStrokeWidth(6);
        mAxisPaint.setAntiAlias(true);

        mTargetIndicatorPaint.setColor(0xFF39FF14); // Neon green
        mTargetIndicatorPaint.setStyle(Paint.Style.STROKE);
        mTargetIndicatorPaint.setStrokeWidth(4);
        mTargetIndicatorPaint.setAntiAlias(true);

        mTextPaint.setColor(0xFF39FF14); // Neon green text
        mTextPaint.setTextSize(40);
        mTextPaint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        mTextPaint.setAntiAlias(true);

        mBgPaint.setColor(0xFF000000); // Black
        mBgPaint.setAlpha(180);         // Semi-transparent
        mBgPaint.setStyle(Paint.Style.FILL);
        mBgPaint.setAntiAlias(true);

        mGridPaint.setColor(0x3F888888); // Semi-transparent grey
        mGridPaint.setStyle(Paint.Style.STROKE);
        mGridPaint.setStrokeWidth(2);
        mGridPaint.setAntiAlias(true);

        mFrustumPaint.setColor(0x7FCCCCCC);
        mFrustumPaint.setStyle(Paint.Style.STROKE);
        mFrustumPaint.setStrokeWidth(3);
        mFrustumPaint.setAntiAlias(true);

        mRayPaint.setColor(0xFFFFD700); // Golden yellow
        mRayPaint.setStyle(Paint.Style.STROKE);
        mRayPaint.setStrokeWidth(4);
        mRayPaint.setAntiAlias(true);
        mRayPaint.setPathEffect(new DashPathEffect(new float[]{15, 10}, 0));

        mTagAxisPaint.setStyle(Paint.Style.STROKE);
        mTagAxisPaint.setStrokeWidth(6);
        mTagAxisPaint.setAntiAlias(true);

        mReticlePaint.setColor(0xFF00E5FF); // Neon cyan
        mReticlePaint.setStyle(Paint.Style.STROKE);
        mReticlePaint.setStrokeWidth(4);
        mReticlePaint.setAntiAlias(true);

        mDotPaint.setColor(0xFF00E5FF);
        mDotPaint.setStyle(Paint.Style.FILL);
        mDotPaint.setAntiAlias(true);

        mLinePaint.setColor(0xFF00E5FF); // Neon cyan
        mLinePaint.setStyle(Paint.Style.STROKE);
        mLinePaint.setStrokeWidth(6);
        mLinePaint.setAntiAlias(true);
        mLinePaint.setPathEffect(new DashPathEffect(new float[]{15, 10}, 0));
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
        mPath.reset();
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                mPath.moveTo(xPointsCanvas[i], yPointsCanvas[i]);
            } else {
                mPath.lineTo(xPointsCanvas[i], yPointsCanvas[i]);
            }
        }
        mPath.close();
        canvas.drawPath(mPath, mFillPaint);

        // Render stroke outline of detections (uniform neon green border)
        canvas.drawPath(mPath, mBorderPaint);

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
                // Front face
                mPath.reset();
                mPath.moveTo(p4[0], p4[1]);
                mPath.lineTo(p5[0], p5[1]);
                mPath.lineTo(p6[0], p6[1]);
                mPath.lineTo(p7[0], p7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Left face
                mPath.reset();
                mPath.moveTo(p0[0], p0[1]);
                mPath.lineTo(p3[0], p3[1]);
                mPath.lineTo(p7[0], p7[1]);
                mPath.lineTo(p4[0], p4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Right face
                mPath.reset();
                mPath.moveTo(p1[0], p1[1]);
                mPath.lineTo(p2[0], p2[1]);
                mPath.lineTo(p6[0], p6[1]);
                mPath.lineTo(p5[0], p5[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Top face
                mPath.reset();
                mPath.moveTo(p0[0], p0[1]);
                mPath.lineTo(p1[0], p1[1]);
                mPath.lineTo(p5[0], p5[1]);
                mPath.lineTo(p4[0], p4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Bottom face
                mPath.reset();
                mPath.moveTo(p3[0], p3[1]);
                mPath.lineTo(p2[0], p2[1]);
                mPath.lineTo(p6[0], p6[1]);
                mPath.lineTo(p7[0], p7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Draw wireframe outlines (edges)
                // Draw back face outline
                canvas.drawLine(p0[0], p0[1], p1[0], p1[1], mEdgePaint);
                canvas.drawLine(p1[0], p1[1], p2[0], p2[1], mEdgePaint);
                canvas.drawLine(p2[0], p2[1], p3[0], p3[1], mEdgePaint);
                canvas.drawLine(p3[0], p3[1], p0[0], p0[1], mEdgePaint);

                // Draw front face outline
                canvas.drawLine(p4[0], p4[1], p5[0], p5[1], mEdgePaint);
                canvas.drawLine(p5[0], p5[1], p6[0], p6[1], mEdgePaint);
                canvas.drawLine(p6[0], p6[1], p7[0], p7[1], mEdgePaint);
                canvas.drawLine(p7[0], p7[1], p4[0], p4[1], mEdgePaint);

                // Draw connecting edges
                canvas.drawLine(p0[0], p0[1], p4[0], p4[1], mEdgePaint);
                canvas.drawLine(p1[0], p1[1], p5[0], p5[1], mEdgePaint);
                canvas.drawLine(p2[0], p2[1], p6[0], p6[1], mEdgePaint);
                canvas.drawLine(p3[0], p3[1], p7[0], p7[1], mEdgePaint);
            }

            // Draw 3D coordinate axes
            float[] origin = project3DPoint(0.0, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] xAxis = project3DPoint(tagSize, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] yAxis = project3DPoint(0.0, tagSize, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);
            float[] zAxis = project3DPoint(0.0, 0.0, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas);

            if (origin != null) {
                // X-axis: Red
                if (xAxis != null) {
                    mAxisPaint.setColor(0xFFFF3F3F);
                    canvas.drawLine(origin[0], origin[1], xAxis[0], xAxis[1], mAxisPaint);
                }
                // Y-axis: Green
                if (yAxis != null) {
                    mAxisPaint.setColor(0xFF3FDF3F);
                    canvas.drawLine(origin[0], origin[1], yAxis[0], yAxis[1], mAxisPaint);
                }
                // Z-axis: Blue
                if (zAxis != null) {
                    mAxisPaint.setColor(0xFF3F3FFF);
                    canvas.drawLine(origin[0], origin[1], zAxis[0], zAxis[1], mAxisPaint);
                }
            }
        }

        // Render target-center indicator crosshair (circle + cross)
        float tagCenterX = (float) (canvas.getWidth() - detection.c[1] * scaleDetectionY);
        float tagCenterY = (float) (detection.c[0] * scaleDetectionX);

        canvas.drawCircle(tagCenterX, tagCenterY, 12, mTargetIndicatorPaint);
        canvas.drawLine(tagCenterX - 18, tagCenterY, tagCenterX + 18, tagCenterY, mTargetIndicatorPaint);
        canvas.drawLine(tagCenterX, tagCenterY - 18, tagCenterX, tagCenterY + 18, mTargetIndicatorPaint);

        // Render Tag ID badge
        mTextPaint.setTextSize(40);
        String badgeText = "ID: " + detection.id;
        if (pose != null) {
            badgeText = String.format("ID: %d (%.2fm)", detection.id, pose.distance);
        }
        float textWidth = mTextPaint.measureText(badgeText);
        Paint.FontMetrics fm = mTextPaint.getFontMetrics();
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

        canvas.drawRoundRect(badgeLeft, badgeTop, badgeRight, badgeBottom, 12, 12, mBgPaint);

        // Draw tag ID text centered
        float textX = tagCenterX - textWidth / 2f;
        float textY = badgeBottom - paddingY - fm.descent;
        canvas.drawText(badgeText, textX, textY, mTextPaint);

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
        for (float xVal = -2.5f; xVal <= 2.5f; xVal += 0.5f) {
            float[] pStart = projectPhysical3DToVirtual(xVal, 0.5, -1.0, width, height);
            float[] pEnd = projectPhysical3DToVirtual(xVal, 0.5, 6.0, width, height);
            if (pStart != null && pEnd != null) {
                canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], mGridPaint);
            }
        }
        for (float zVal = -1.0f; zVal <= 6.0f; zVal += 0.5f) {
            float[] pStart = projectPhysical3DToVirtual(-2.5, 0.5, zVal, width, height);
            float[] pEnd = projectPhysical3DToVirtual(2.5, 0.5, zVal, width, height);
            if (pStart != null && pEnd != null) {
                canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], mGridPaint);
            }
        }

        // 2. Draw camera axes and frustum at the origin (0,0,0)
        float[] origin = projectPhysical3DToVirtual(0, 0, 0, width, height);
        float[] xAxisCam = projectPhysical3DToVirtual(0.25, 0, 0, width, height);
        float[] yAxisCam = projectPhysical3DToVirtual(0, 0.25, 0, width, height);
        float[] zAxisCam = projectPhysical3DToVirtual(0, 0, 0.25, width, height);

        if (origin != null) {
            if (xAxisCam != null) {
                mAxisPaint.setColor(0xFFFF3F3F);
                canvas.drawLine(origin[0], origin[1], xAxisCam[0], xAxisCam[1], mAxisPaint);
            }
            if (yAxisCam != null) {
                mAxisPaint.setColor(0xFF3FDF3F);
                canvas.drawLine(origin[0], origin[1], yAxisCam[0], yAxisCam[1], mAxisPaint);
            }
            if (zAxisCam != null) {
                mAxisPaint.setColor(0xFF3F3FFF);
                canvas.drawLine(origin[0], origin[1], zAxisCam[0], zAxisCam[1], mAxisPaint);
            }
        }

        float[] pf0 = projectPhysical3DToVirtual(-0.18, -0.13, 0.35, width, height);
        float[] pf1 = projectPhysical3DToVirtual(0.18, -0.13, 0.35, width, height);
        float[] pf2 = projectPhysical3DToVirtual(0.18, 0.13, 0.35, width, height);
        float[] pf3 = projectPhysical3DToVirtual(-0.18, 0.13, 0.35, width, height);

        if (origin != null && pf0 != null && pf1 != null && pf2 != null && pf3 != null) {
            canvas.drawLine(origin[0], origin[1], pf0[0], pf0[1], mFrustumPaint);
            canvas.drawLine(origin[0], origin[1], pf1[0], pf1[1], mFrustumPaint);
            canvas.drawLine(origin[0], origin[1], pf2[0], pf2[1], mFrustumPaint);
            canvas.drawLine(origin[0], origin[1], pf3[0], pf3[1], mFrustumPaint);

            canvas.drawLine(pf0[0], pf0[1], pf1[0], pf1[1], mFrustumPaint);
            canvas.drawLine(pf1[0], pf1[1], pf2[0], pf2[1], mFrustumPaint);
            canvas.drawLine(pf2[0], pf2[1], pf3[0], pf3[1], mFrustumPaint);
            canvas.drawLine(pf3[0], pf3[1], pf0[0], pf0[1], mFrustumPaint);
        }

        // 3. Draw detected tags (axes, wireframe cubes, tracking rays, labels)
        for (Pose3D pose : poses) {
            float[] tagCenter = projectPhysical3DToVirtual(pose.tx, pose.ty, pose.tz, width, height);
            if (origin != null && tagCenter != null) {
                canvas.drawLine(origin[0], origin[1], tagCenter[0], tagCenter[1], mRayPaint);
            }

            double L = tagSize;
            float[] pxAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r1[0], pose.ty + L * pose.r1[1], pose.tz + L * pose.r1[2], width, height);
            float[] pyAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r2[0], pose.ty + L * pose.r2[1], pose.tz + L * pose.r2[2], width, height);
            float[] pzAxis = projectPhysical3DToVirtual(pose.tx + L * pose.r3[0], pose.ty + L * pose.r3[1], pose.tz + L * pose.r3[2], width, height);

            if (tagCenter != null) {
                if (pxAxis != null) {
                    mTagAxisPaint.setColor(0xFFFF3F3F);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pxAxis[0], pxAxis[1], mTagAxisPaint);
                }
                if (pyAxis != null) {
                    mTagAxisPaint.setColor(0xFF3FDF3F);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pyAxis[0], pyAxis[1], mTagAxisPaint);
                }
                if (pzAxis != null) {
                    mTagAxisPaint.setColor(0xFF3F3FFF);
                    canvas.drawLine(tagCenter[0], tagCenter[1], pzAxis[0], pzAxis[1], mTagAxisPaint);
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

                mPath.reset();
                mPath.moveTo(c4[0], c4[1]);
                mPath.lineTo(c5[0], c5[1]);
                mPath.lineTo(c6[0], c6[1]);
                mPath.lineTo(c7[0], c7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                mPath.reset();
                mPath.moveTo(c0[0], c0[1]);
                mPath.lineTo(c3[0], c3[1]);
                mPath.lineTo(c7[0], c7[1]);
                mPath.lineTo(c4[0], c4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                mPath.reset();
                mPath.moveTo(c1[0], c1[1]);
                mPath.lineTo(c2[0], c2[1]);
                mPath.lineTo(c6[0], c6[1]);
                mPath.lineTo(c5[0], c5[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                mPath.reset();
                mPath.moveTo(c0[0], c0[1]);
                mPath.lineTo(c1[0], c1[1]);
                mPath.lineTo(c5[0], c5[1]);
                mPath.lineTo(c4[0], c4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                mPath.reset();
                mPath.moveTo(c3[0], c3[1]);
                mPath.lineTo(c2[0], c2[1]);
                mPath.lineTo(c6[0], c6[1]);
                mPath.lineTo(c7[0], c7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                canvas.drawLine(c0[0], c0[1], c1[0], c1[1], mEdgePaint);
                canvas.drawLine(c1[0], c1[1], c2[0], c2[1], mEdgePaint);
                canvas.drawLine(c2[0], c2[1], c3[0], c3[1], mEdgePaint);
                canvas.drawLine(c3[0], c3[1], c0[0], c0[1], mEdgePaint);

                canvas.drawLine(c4[0], c4[1], c5[0], c5[1], mEdgePaint);
                canvas.drawLine(c5[0], c5[1], c6[0], c6[1], mEdgePaint);
                canvas.drawLine(c6[0], c6[1], c7[0], c7[1], mEdgePaint);
                canvas.drawLine(c7[0], c7[1], c4[0], c4[1], mEdgePaint);

                canvas.drawLine(c0[0], c0[1], c4[0], c4[1], mEdgePaint);
                canvas.drawLine(c1[0], c1[1], c5[0], c5[1], mEdgePaint);
                canvas.drawLine(c2[0], c2[1], c6[0], c6[1], mEdgePaint);
                canvas.drawLine(c3[0], c3[1], c7[0], c7[1], mEdgePaint);
            }

            float[] badgePos = projectPhysical3DToVirtual(pose.tx, pose.ty - 0.15, pose.tz, width, height);
            if (badgePos != null) {
                mTextPaint.setTextSize(36);
                String badgeText = String.format("ID: %d (%.2fm)", pose.id, pose.distance);
                float textWidth = mTextPaint.measureText(badgeText);
                Paint.FontMetrics fm = mTextPaint.getFontMetrics();
                float textHeight = fm.descent - fm.ascent;

                float paddingX = 12;
                float paddingY = 6;
                float badgeLeft = badgePos[0] - textWidth / 2f - paddingX;
                float badgeRight = badgePos[0] + textWidth / 2f + paddingX;
                float badgeTop = badgePos[1] - textHeight / 2f - paddingY;
                float badgeBottom = badgePos[1] + textHeight / 2f + paddingY;

                canvas.drawRoundRect(badgeLeft, badgeTop, badgeRight, badgeBottom, 8, 8, mBgPaint);

                float textX = badgePos[0] - textWidth / 2f;
                float textY = badgeBottom - paddingY - fm.descent;
                canvas.drawText(badgeText, textX, textY, mTextPaint);
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

            BufferReleaseListener listener = mBufferReleaseListener;
            if (listener != null) {
                listener.onBufferReleased(data);
            }

            mLastDetectLatency = (System.currentTimeMillis() - mLastEnqueueFrameTime);
        }
    }
}