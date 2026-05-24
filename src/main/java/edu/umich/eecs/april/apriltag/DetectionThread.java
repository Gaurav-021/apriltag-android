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
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.RectF;

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

    public static final int MODE_2D = 0;
    public static final int MODE_3D = 1;
    public static final int MODE_FRC = 2;

    private volatile int mRenderMode = MODE_2D;
    private Bitmap mFrcFieldBitmap = null;
    private volatile float mVirtualYaw = -0.5f; // approx -30 degrees
    private volatile float mVirtualPitch = 0.4f; // approx 23 degrees
    // Adaptive ROI Tracking state
    private boolean mRoiActive = false;
    private int mRoiLastTagId = -1;
    private int mRoiCenterX = 0;
    private int mRoiCenterY = 0;
    private int mRoiHalfSize = 120;

    private float mFocalX = 0.0f;
    private float mFocalY = 0.0f;
    private float mFocalZ = 1.5f;
    private float mVirtualDistance = 2.0f;

    public void setRenderMode(int mode) {
        mRenderMode = mode;
    }

    public void updateVirtualCameraOrbit(float dYaw, float dPitch) {
        mVirtualYaw += dYaw;
        mVirtualPitch += dPitch;

        // Clip pitch to prevent flipping upside down or going underground
        float maxPitch = (float) Math.toRadians(85);
        float minPitch = (float) Math.toRadians(-5);
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

    // Pre-allocated arrays and objects to avoid allocations in frame loops
    private final float[] mDetectionXPoints = new float[4];
    private final float[] mDetectionYPoints = new float[4];
    private final Path mFrcConePath = new Path();
    private final Paint mFrcConePaint = new Paint();
    private final Paint mFrcConeEdgePaint = new Paint();
    private final Paint mFrcCamPaint = new Paint();
    private final Paint mFrcCamStrokePaint = new Paint();
    private final Paint mFrcYawLinePaint = new Paint();
    private final RectF mFrcDestRect = new RectF();

    // Pre-allocated lists to avoid frame-level allocations
    private final ArrayList<Pose3D> mFrcPoses = new ArrayList<>();
    private final ArrayList<Pose3D> m3dPoses = new ArrayList<>();
    private final ArrayList<Pose3D> mDefaultPoses = new ArrayList<>();

    // Pre-allocated coordinate projection points to avoid float[] allocations
    private final float[] mP0 = new float[2];
    private final float[] mP1 = new float[2];
    private final float[] mP2 = new float[2];
    private final float[] mP3 = new float[2];
    private final float[] mP4 = new float[2];
    private final float[] mP5 = new float[2];
    private final float[] mP6 = new float[2];
    private final float[] mP7 = new float[2];
    private final float[] mOriginPoint = new float[2];
    private final float[] mXAxisPoint = new float[2];
    private final float[] mYAxisPoint = new float[2];
    private final float[] mZAxisPoint = new float[2];

    // Helper arrays for pose estimation
    private final double[] mXNorm = new double[4];
    private final double[] mYNorm = new double[4];
    private final double[] mXSrc = new double[4];
    private final double[] mYSrc = new double[4];
    private final double[][] mMatrixA = new double[8][8];
    private final double[] mVectorB = new double[8];

    // Cached configuration/preferences
    private boolean mFrcMode = false;
    private double mTagSize = 0.165;
    private boolean mCalOverride = false;
    private double mCustomFx = 600.0;
    private double mCustomFy = 600.0;
    private double mCustomCx = 0.0;
    private double mCustomCy = 0.0;

    private final SharedPreferences.OnSharedPreferenceChangeListener mPrefChangeListener = 
        new SharedPreferences.OnSharedPreferenceChangeListener() {
            @Override
            public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
                loadPreferences();
            }
        };

    private void loadPreferences() {
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mTextureView.getContext());
        mFrcMode = sharedPreferences.getBoolean("frc_mode", false);
        
        if (mFrcMode) {
            mTagSize = 0.1651;
        } else {
            String sizeStr = sharedPreferences.getString("apriltag_size", "0.165");
            try {
                mTagSize = Double.parseDouble(sizeStr);
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid apriltag_size setting, defaulting to 0.165");
                mTagSize = 0.165;
            }
        }

        mCalOverride = sharedPreferences.getBoolean("calibration_override", false);
        if (mCalOverride) {
            try {
                mCustomFx = Double.parseDouble(sharedPreferences.getString("calibration_fx", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fx, using default 600.0");
                mCustomFx = 600.0;
            }
            try {
                mCustomFy = Double.parseDouble(sharedPreferences.getString("calibration_fy", "600.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_fy, using default 600.0");
                mCustomFy = 600.0;
            }
            try {
                mCustomCx = Double.parseDouble(sharedPreferences.getString("calibration_cx", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cx, using default 0.0");
                mCustomCx = 0.0;
            }
            try {
                mCustomCy = Double.parseDouble(sharedPreferences.getString("calibration_cy", "0.0"));
            } catch (NumberFormatException e) {
                Log.e(TAG, "Invalid calibration_cy, using default 0.0");
                mCustomCy = 0.0;
            }
        }
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

        // Destination points normalized (using pre-allocated arrays)
        double[] xNorm = mXNorm;
        double[] yNorm = mYNorm;
        for (int i = 0; i < 4; i++) {
            xNorm[i] = (points[i * 2] - cx) / fx;
            yNorm[i] = (points[i * 2 + 1] - cy) / fy;
        }

        // Source points on the tag plane
        double s2 = tagSize / 2.0;
        double[] xSrc = mXSrc;
        double[] ySrc = mYSrc;
        xSrc[0] = -s2; xSrc[1] = s2; xSrc[2] = s2; xSrc[3] = -s2;
        ySrc[0] = s2;  ySrc[1] = s2;  ySrc[2] = -s2; ySrc[3] = -s2;

        // A h = B
        double[][] A = mMatrixA;
        double[] B = mVectorB;
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

    private float[] projectFieldPointToVirtual(double xField, double yField, double zField, int width, int height) {
        return projectPhysical3DToVirtual(xField, -zField, yField, width, height);
    }

    private float[] projectCameraPointInFRC(double xCam, double yCam, double zCam, 
                                            double cx_f, double cy_f, double cz_f,
                                            double[][] R_f_c, int width, int height) {
        double xField = R_f_c[0][0]*xCam + R_f_c[0][1]*yCam + R_f_c[0][2]*zCam + cx_f;
        double yField = R_f_c[1][0]*xCam + R_f_c[1][1]*yCam + R_f_c[1][2]*zCam + cy_f;
        double zField = R_f_c[2][0]*xCam + R_f_c[2][1]*yCam + R_f_c[2][2]*zCam + cz_f;
        return projectFieldPointToVirtual(xField, yField, zField, width, height);
    }

    private boolean project3DPoint(double xTag, double yTag, double zTag,
                                   double[] r1, double[] r2, double[] r3, double[] t,
                                   double fx, double fy, double cx, double cy,
                                   float scaleDetectionX, float scaleDetectionY, Canvas canvas,
                                   float[] outPoint) {
        double xCam = r1[0] * xTag + r2[0] * yTag + r3[0] * zTag + t[0];
        double yCam = r1[1] * xTag + r2[1] * yTag + r3[1] * zTag + t[1];
        double zCam = r1[2] * xTag + r2[2] * yTag + r3[2] * zTag + t[2];

        if (zCam <= 1e-3) {
            return false; // Behind camera
        }

        double xPix = xCam * (fx / zCam) + cx;
        double yPix = yCam * (fy / zCam) + cy;

        outPoint[0] = (float) (canvas.getWidth() - yPix * scaleDetectionY);
        outPoint[1] = (float) (xPix * scaleDetectionX);

        return true;
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

        // Initialize reusable FRC map paints
        mFrcConePaint.setColor(0x40FF9800); // 25% transparent orange
        mFrcConePaint.setStyle(Paint.Style.FILL);
        mFrcConePaint.setAntiAlias(true);

        mFrcConeEdgePaint.setColor(0x80FF9800); // 50% transparent orange edge
        mFrcConeEdgePaint.setStyle(Paint.Style.STROKE);
        mFrcConeEdgePaint.setStrokeWidth(2f);
        mFrcConeEdgePaint.setAntiAlias(true);

        mFrcCamPaint.setColor(0xFFFF5722); // Deep Orange
        mFrcCamPaint.setStyle(Paint.Style.FILL);
        mFrcCamPaint.setAntiAlias(true);

        mFrcCamStrokePaint.setColor(Color.WHITE);
        mFrcCamStrokePaint.setStyle(Paint.Style.STROKE);
        mFrcCamStrokePaint.setStrokeWidth(3f);
        mFrcCamStrokePaint.setAntiAlias(true);

        mFrcYawLinePaint.setColor(0xFFFF9800); // Orange
        mFrcYawLinePaint.setStyle(Paint.Style.STROKE); // Ensure STROKE style for lines
        mFrcYawLinePaint.setStrokeWidth(5f);
        mFrcYawLinePaint.setAntiAlias(true);

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

        // Initialize and register preference listener
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mTextureView.getContext());
        sharedPreferences.registerOnSharedPreferenceChangeListener(mPrefChangeListener);
        loadPreferences();
    }

    public void destroy() {
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mTextureView.getContext());
        sharedPreferences.unregisterOnSharedPreferenceChangeListener(mPrefChangeListener);

        mCameraFrameQueue.clear();
        mCameraFrameQueue = null;
        if (mFrcFieldBitmap != null) {
            mFrcFieldBitmap.recycle();
            mFrcFieldBitmap = null;
        }
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

    private byte[] cropYPlane(byte[] fullYBytes, int fullW, int fullH, int cropX, int cropY, int cropW, int cropH) {
        byte[] cropped = new byte[cropW * cropH];
        for (int y = 0; y < cropH; y++) {
            int srcPos = (cropY + y) * fullW + cropX;
            int destPos = y * cropW;
            System.arraycopy(fullYBytes, srcPos, cropped, destPos, cropW);
        }
        return cropped;
    }

    private ArrayList<ApriltagDetection> processCameraFrame(byte[] data, Camera.Size cameraSize)  {
        try {
            return ApriltagNative.apriltag_detect_yuv_flat_decoded(data, cameraSize.width, cameraSize.height);
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

        // Convert detection points to canvas points (using pre-allocated arrays)
        float[] xPointsCanvas = mDetectionXPoints;
        float[] yPointsCanvas = mDetectionYPoints;
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

        // Retrieve AprilTag Size from cached settings
        double tagSize = mTagSize;

        // Check for manual camera calibration settings override (using cached settings)
        double cx = mCameraSize.width / 2.0;
        double cy = mCameraSize.height / 2.0;
        double fx, fy;

        if (mCalOverride) {
            fx = mCustomFx;
            fy = mCustomFy;
            if (mCustomCx > 0.0) {
                cx = mCustomCx;
            }
            if (mCustomCy > 0.0) {
                cy = mCustomCy;
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

            // Project 8 corners of the 3D box using pre-allocated arrays
            // Back face (on the tag): z = 0
            boolean p0_ok = project3DPoint(-s2, s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP0);
            boolean p1_ok = project3DPoint(s2, s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP1);
            boolean p2_ok = project3DPoint(s2, -s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP2);
            boolean p3_ok = project3DPoint(-s2, -s2, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP3);

            // Front face (projected out towards the camera): z = -tagSize
            boolean p4_ok = project3DPoint(-s2, s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP4);
            boolean p5_ok = project3DPoint(s2, s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP5);
            boolean p6_ok = project3DPoint(s2, -s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP6);
            boolean p7_ok = project3DPoint(-s2, -s2, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mP7);

            if (p0_ok && p1_ok && p2_ok && p3_ok &&
                p4_ok && p5_ok && p6_ok && p7_ok) {

                // Draw faces of the 3D box with light transparent fill
                // Front face
                mPath.reset();
                mPath.moveTo(mP4[0], mP4[1]);
                mPath.lineTo(mP5[0], mP5[1]);
                mPath.lineTo(mP6[0], mP6[1]);
                mPath.lineTo(mP7[0], mP7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Left face
                mPath.reset();
                mPath.moveTo(mP0[0], mP0[1]);
                mPath.lineTo(mP3[0], mP3[1]);
                mPath.lineTo(mP7[0], mP7[1]);
                mPath.lineTo(mP4[0], mP4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Right face
                mPath.reset();
                mPath.moveTo(mP1[0], mP1[1]);
                mPath.lineTo(mP2[0], mP2[1]);
                mPath.lineTo(mP6[0], mP6[1]);
                mPath.lineTo(mP5[0], mP5[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Top face
                mPath.reset();
                mPath.moveTo(mP0[0], mP0[1]);
                mPath.lineTo(mP1[0], mP1[1]);
                mPath.lineTo(mP5[0], mP5[1]);
                mPath.lineTo(mP4[0], mP4[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Bottom face
                mPath.reset();
                mPath.moveTo(mP3[0], mP3[1]);
                mPath.lineTo(mP2[0], mP2[1]);
                mPath.lineTo(mP6[0], mP6[1]);
                mPath.lineTo(mP7[0], mP7[1]);
                mPath.close();
                canvas.drawPath(mPath, mFacePaint);

                // Draw wireframe outlines (edges)
                // Draw back face outline
                canvas.drawLine(mP0[0], mP0[1], mP1[0], mP1[1], mEdgePaint);
                canvas.drawLine(mP1[0], mP1[1], mP2[0], mP2[1], mEdgePaint);
                canvas.drawLine(mP2[0], mP2[1], mP3[0], mP3[1], mEdgePaint);
                canvas.drawLine(mP3[0], mP3[1], mP0[0], mP0[1], mEdgePaint);

                // Draw front face outline
                canvas.drawLine(mP4[0], mP4[1], mP5[0], mP5[1], mEdgePaint);
                canvas.drawLine(mP5[0], mP5[1], mP6[0], mP6[1], mEdgePaint);
                canvas.drawLine(mP6[0], mP6[1], mP7[0], mP7[1], mEdgePaint);
                canvas.drawLine(mP7[0], mP7[1], mP4[0], mP4[1], mEdgePaint);

                // Draw connecting edges
                canvas.drawLine(mP0[0], mP0[1], mP4[0], mP4[1], mEdgePaint);
                canvas.drawLine(mP1[0], mP1[1], mP5[0], mP5[1], mEdgePaint);
                canvas.drawLine(mP2[0], mP2[1], mP6[0], mP6[1], mEdgePaint);
                canvas.drawLine(mP3[0], mP3[1], mP7[0], mP7[1], mEdgePaint);
            }

            // Draw 3D coordinate axes
            boolean origin_ok = project3DPoint(0.0, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mOriginPoint);
            boolean xAxis_ok = project3DPoint(tagSize, 0.0, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mXAxisPoint);
            boolean yAxis_ok = project3DPoint(0.0, tagSize, 0.0, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mYAxisPoint);
            boolean zAxis_ok = project3DPoint(0.0, 0.0, -tagSize, pose.r1, pose.r2, pose.r3, pose.t, fx, fy, cx, cy, scaleDetectionX, scaleDetectionY, canvas, mZAxisPoint);

            if (origin_ok) {
                // X-axis: Red
                if (xAxis_ok) {
                    mAxisPaint.setColor(0xFFFF3F3F);
                    canvas.drawLine(mOriginPoint[0], mOriginPoint[1], mXAxisPoint[0], mXAxisPoint[1], mAxisPaint);
                }
                // Y-axis: Green
                if (yAxis_ok) {
                    mAxisPaint.setColor(0xFF3FDF3F);
                    canvas.drawLine(mOriginPoint[0], mOriginPoint[1], mYAxisPoint[0], mYAxisPoint[1], mAxisPaint);
                }
                // Z-axis: Blue
                if (zAxis_ok) {
                    mAxisPaint.setColor(0xFF3F3FFF);
                    canvas.drawLine(mOriginPoint[0], mOriginPoint[1], mZAxisPoint[0], mZAxisPoint[1], mAxisPaint);
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

    private static class MultiTagResult {
        public FRCTagLayout.CameraPose cameraPose;
        public double[][] R_f_c;
        public int closestTagId;
    }

    private MultiTagResult computeMultiTagCameraPose(ArrayList<Pose3D> poses) {
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;
        
        double[][] R_sum = new double[3][3];
        double totalWeight = 0;
        int validCount = 0;
        int closestTagId = -1;
        double minCamDist = Double.MAX_VALUE;

        for (Pose3D pose : poses) {
            // Convert pose rotation columns from detector convention to FRC/WPILib convention:
            // r1_frc = -pose.r3 (tag normal)
            // r2_frc = pose.r1  (horizontal left)
            // r3_frc = -pose.r2 (vertical up)
            double[] r1_frc = new double[]{-pose.r3[0], -pose.r3[1], -pose.r3[2]};
            double[] r2_frc = new double[]{pose.r1[0], pose.r1[1], pose.r1[2]};
            double[] r3_frc = new double[]{-pose.r2[0], -pose.r2[1], -pose.r2[2]};

            FRCTagLayout.CameraPose cameraPose = FRCTagLayout.computeCameraPoseOnField(
                    pose.id, r1_frc, r2_frc, r3_frc, pose.t);
            if (cameraPose != null) {
                double dist = Math.max(0.1, pose.distance);
                double weight = 1.0 / (dist * dist);

                sumX += cameraPose.x * weight;
                sumY += cameraPose.y * weight;
                sumZ += cameraPose.z * weight;

                // Reconstruct R_f_c for this detection using FRC convention columns
                double[][] T_field_tag = FRCTagLayout.TAG_TRANSFORMS[pose.id];
                double[][] R_f_c_temp = new double[3][3];
                for (int i = 0; i < 3; i++) {
                    R_f_c_temp[i][0] = T_field_tag[i][0]*r1_frc[0] + T_field_tag[i][1]*r2_frc[0] + T_field_tag[i][2]*r3_frc[0];
                    R_f_c_temp[i][1] = T_field_tag[i][0]*r1_frc[1] + T_field_tag[i][1]*r2_frc[1] + T_field_tag[i][2]*r3_frc[1];
                    R_f_c_temp[i][2] = T_field_tag[i][0]*r1_frc[2] + T_field_tag[i][1]*r2_frc[2] + T_field_tag[i][2]*r3_frc[2];
                }

                // Add to R_sum
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        R_sum[i][j] += R_f_c_temp[i][j] * weight;
                    }
                }

                totalWeight += weight;
                validCount++;

                if (pose.distance < minCamDist) {
                    minCamDist = pose.distance;
                    closestTagId = pose.id;
                }
            }
        }

        if (validCount == 0) {
            return null;
        }

        double avgX = sumX / totalWeight;
        double avgY = sumY / totalWeight;
        double avgZ = sumZ / totalWeight;

        // Orthogonalize R_sum using Gram-Schmidt
        // Column 0
        double len0 = Math.sqrt(R_sum[0][0]*R_sum[0][0] + R_sum[1][0]*R_sum[1][0] + R_sum[2][0]*R_sum[2][0]);
        if (len0 < 1e-6) len0 = 1.0;
        double[] r0 = new double[]{R_sum[0][0]/len0, R_sum[1][0]/len0, R_sum[2][0]/len0};
        
        // Column 1
        double dot01 = r0[0]*R_sum[0][1] + r0[1]*R_sum[1][1] + r0[2]*R_sum[2][1];
        double[] r1_proj = new double[]{
            R_sum[0][1] - dot01*r0[0],
            R_sum[1][1] - dot01*r0[1],
            R_sum[2][1] - dot01*r0[2]
        };
        double len1 = Math.sqrt(r1_proj[0]*r1_proj[0] + r1_proj[1]*r1_proj[1] + r1_proj[2]*r1_proj[2]);
        if (len1 < 1e-6) len1 = 1.0;
        double[] r1 = new double[]{r1_proj[0]/len1, r1_proj[1]/len1, r1_proj[2]/len1};
        
        // Column 2 = Col 0 x Col 1
        double[] r2 = new double[]{
            r0[1]*r1[2] - r0[2]*r1[1],
            r0[2]*r1[0] - r0[0]*r1[2],
            r0[0]*r1[1] - r0[1]*r1[0]
        };

        double[][] R_f_c = new double[3][3];
        R_f_c[0][0] = r0[0]; R_f_c[1][0] = r0[1]; R_f_c[2][0] = r0[2];
        R_f_c[0][1] = r1[0]; R_f_c[1][1] = r1[1]; R_f_c[2][1] = r1[2];
        R_f_c[0][2] = r2[0]; R_f_c[1][2] = r2[1]; R_f_c[2][2] = r2[2];

        // Extract Roll, Pitch, Yaw from final R_f_c relative to FRC horizontal reference orientation
        double pitch = Math.asin(-R_f_c[2][2]);
        double roll, yaw;
        if (Math.abs(R_f_c[2][2]) < 0.999) {
            roll = Math.atan2(-R_f_c[2][0], -R_f_c[2][1]);
            yaw = Math.atan2(R_f_c[1][2], R_f_c[0][2]);
        } else {
            roll = 0.0;
            yaw = Math.atan2(-R_f_c[0][1], R_f_c[0][0]);
        }

        FRCTagLayout.CameraPose cameraPose = new FRCTagLayout.CameraPose(
                avgX, avgY, avgZ, Math.toDegrees(roll), Math.toDegrees(pitch), Math.toDegrees(yaw));

        MultiTagResult result = new MultiTagResult();
        result.cameraPose = cameraPose;
        result.R_f_c = R_f_c;
        result.closestTagId = closestTagId;
        return result;
    }

    private void renderFRCView(ArrayList<ApriltagDetection> detections, Canvas canvas) {
        if (mFrcFieldBitmap == null) {
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inScaled = false;
            mFrcFieldBitmap = BitmapFactory.decodeResource(mTextureView.getContext().getResources(), R.drawable.frc_field_2026, options);
        }

        canvas.drawColor(0xFF1E1E1E); // RViz dark grey / dark field background

        int viewWidth = canvas.getWidth();
        int viewHeight = canvas.getHeight();
        
        float drawWidth = viewWidth;
        float drawHeight = viewHeight;
        float dx = 0;
        float dy = 0;

        if (mFrcFieldBitmap != null) {
            int imgWidth = mFrcFieldBitmap.getWidth();
            int imgHeight = mFrcFieldBitmap.getHeight();
            float scale = Math.min((float) viewWidth / imgWidth, (float) viewHeight / imgHeight);
            drawWidth = imgWidth * scale;
            drawHeight = imgHeight * scale;
            dx = (viewWidth - drawWidth) / 2f;
            dy = (viewHeight - drawHeight) / 2f;

            mFrcDestRect.set(dx, dy, dx + drawWidth, dy + drawHeight);
            canvas.drawBitmap(mFrcFieldBitmap, null, mFrcDestRect, null);
        }

        double tagSize = 0.1651; // Locked for FRC

        double cx = mCameraSize != null ? mCameraSize.width / 2.0 : viewWidth / 2.0;
        double cy = mCameraSize != null ? mCameraSize.height / 2.0 : viewHeight / 2.0;
        double fx, fy;

        if (mCalOverride) {
            fx = mCustomFx;
            fy = mCustomFy;
            if (mCustomCx > 0.0) cx = mCustomCx;
            if (mCustomCy > 0.0) cy = mCustomCy;
        } else {
            fx = cx / Math.tan(Math.toRadians(mFovH / 2.0));
            fy = cy / Math.tan(Math.toRadians(mFovV / 2.0));
        }

        mFrcPoses.clear();
        for (ApriltagDetection detection : detections) {
            Pose3D pose = estimatePose(detection, tagSize, fx, fy, cx, cy);
            if (pose != null) {
                mFrcPoses.add(pose);
            }
        }

        MultiTagResult multiTagResult = computeMultiTagCameraPose(mFrcPoses);
        FRCTagLayout.CameraPose cameraPose = (multiTagResult != null) ? multiTagResult.cameraPose : null;
        int targetId = (multiTagResult != null) ? multiTagResult.closestTagId : -1;

        // Draw camera estimated pose if available
        if (cameraPose != null) {
            // Horizontally flip X axis and yaw to match the background field image (where Red is on the Left and Blue is on the Right)
            double x_field = -cameraPose.x;
            double y_field = cameraPose.y;
            double yaw_field = 180.0 - cameraPose.yaw; // in degrees

            // Normalization equations
            double normX = (x_field - (-8.259)) / 16.518;
            double normY = (4.0215 - y_field) / 8.043;

            // Constrain normX and normY to [0, 1] range to avoid drawing outside the field boundaries
            normX = Math.max(0.0, Math.min(1.0, normX));
            normY = Math.max(0.0, Math.min(1.0, normY));

            double ratioX = (254.0 + normX * (3951.0 - 254.0)) / 4206.0;
            double ratioY = (121.0 + normY * (1917.0 - 121.0)) / 2038.0;

            float screenX = dx + (float) ratioX * drawWidth;
            float screenY = dy + (float) ratioY * drawHeight;

            // Draw FOV cone
            mFrcConePath.reset();
            mFrcConePath.moveTo(screenX, screenY);
            
            float coneLength = 80f; // length of FOV cone on screen
            double leftAngle = Math.toRadians(yaw_field + mFovH / 2.0);
            double rightAngle = Math.toRadians(yaw_field - mFovH / 2.0);
            
            float leftX = screenX + (float) (coneLength * Math.cos(leftAngle));
            float leftY = screenY - (float) (coneLength * Math.sin(leftAngle));
            
            float rightX = screenX + (float) (coneLength * Math.cos(rightAngle));
            float rightY = screenY - (float) (coneLength * Math.sin(rightAngle));
            
            mFrcConePath.lineTo(leftX, leftY);
            mFrcConePath.lineTo(rightX, rightY);
            mFrcConePath.close();
            
            canvas.drawPath(mFrcConePath, mFrcConePaint);
            canvas.drawPath(mFrcConePath, mFrcConeEdgePaint);

            // Draw camera circle pointer
            canvas.drawCircle(screenX, screenY, 15f, mFrcCamPaint);
            canvas.drawCircle(screenX, screenY, 15f, mFrcCamStrokePaint);

            // Draw line in the yaw direction
            double yawRad = Math.toRadians(yaw_field);
            float pointerLength = 40f;
            float endX = screenX + (float) (pointerLength * Math.cos(yawRad));
            float endY = screenY - (float) (pointerLength * Math.sin(yawRad));
            
            canvas.drawLine(screenX, screenY, endX, endY, mFrcYawLinePaint);
        }

        // Update telemetry text view
        if (mPoseTextView != null) {
            final String telemetry;
            if (cameraPose != null) {
                telemetry = String.format("Target ID: %d (FRC Field Multi-Tag)\n" +
                                "Field Pose: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                        targetId, cameraPose.x, cameraPose.y, cameraPose.z,
                        cameraPose.roll, cameraPose.pitch, cameraPose.yaw);
            } else if (mFrcPoses.size() > 0) {
                // Find closest pose for fallback telemetry
                Pose3D closestPose = null;
                double minDistance = Double.MAX_VALUE;
                for (Pose3D pose : mFrcPoses) {
                    if (pose.distance < minDistance) {
                        minDistance = pose.distance;
                        closestPose = pose;
                    }
                }
                telemetry = String.format("Target ID: %d (Invalid FRC Tag)\n" +
                                "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                        closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                        closestPose.roll, closestPose.pitch, closestPose.yaw);
            } else {
                telemetry = "No Target";
            }

            mPoseTextView.post(new Runnable() {
                @Override
                public void run() {
                    mPoseTextView.setText(telemetry);
                }
            });
        }
    }

    private void render3DView(ArrayList<ApriltagDetection> detections, Canvas canvas) {
        canvas.drawColor(0xFF1E1E1E); // RViz dark grey

        int width = canvas.getWidth();
        int height = canvas.getHeight();

        double tagSize = mTagSize;
        double cx = mCameraSize != null ? mCameraSize.width / 2.0 : width / 2.0;
        double cy = mCameraSize != null ? mCameraSize.height / 2.0 : height / 2.0;
        double fx, fy;

        if (mCalOverride) {
            fx = mCustomFx;
            fy = mCustomFy;
            if (mCustomCx > 0.0) {
                cx = mCustomCx;
            }
            if (mCustomCy > 0.0) {
                cy = mCustomCy;
            }
        } else {
            fx = cx / Math.tan(Math.toRadians(mFovH / 2.0));
            fy = cy / Math.tan(Math.toRadians(mFovV / 2.0));
        }

        double sumX = 0, sumY = 0, sumZ = 0;
        int validCount = 0;

        m3dPoses.clear();

        for (ApriltagDetection detection : detections) {
            Pose3D pose = estimatePose(detection, tagSize, fx, fy, cx, cy);
            if (pose != null) {
                m3dPoses.add(pose);
                sumX += pose.tx;
                sumY += pose.ty;
                sumZ += pose.tz;
                validCount++;
            }
        }

        double camX_field = 0.0;
        double camY_field = 0.0;
        double camZ_field = 0.0;
        double[][] R_f_c = null;
        boolean cameraEstimated = false;
        MultiTagResult multiTagResult = null;
        Pose3D closestPose = null;

        if (m3dPoses.size() > 0) {
            double minCamDist = Double.MAX_VALUE;
            for (Pose3D pose : m3dPoses) {
                if (pose.distance < minCamDist) {
                    minCamDist = pose.distance;
                    closestPose = pose;
                }
            }
        }

        if (mFrcMode && m3dPoses.size() > 0) {
            multiTagResult = computeMultiTagCameraPose(m3dPoses);
            if (multiTagResult != null) {
                camX_field = multiTagResult.cameraPose.x;
                camY_field = multiTagResult.cameraPose.y;
                camZ_field = multiTagResult.cameraPose.z;
                R_f_c = multiTagResult.R_f_c;
                cameraEstimated = true;
            }
        }

        double targetFocalX = 0.0;
        double targetFocalY = 0.0;
        double targetFocalZ = 1.5;
        double targetDistance = 2.0;

        if (mFrcMode) {
            if (cameraEstimated) {
                targetFocalX = camX_field;
                targetFocalY = -camZ_field;
                targetFocalZ = camY_field;
                targetDistance = 3.0;
            } else {
                targetFocalX = 0.0;
                targetFocalY = 0.0;
                targetFocalZ = 0.0;
                targetDistance = 12.0;
            }
        } else {
            if (validCount > 0) {
                targetFocalX = sumX / validCount;
                targetFocalY = sumY / validCount;
                targetFocalZ = sumZ / validCount;
                double avgDistance = Math.sqrt(targetFocalX * targetFocalX + targetFocalY * targetFocalY + targetFocalZ * targetFocalZ);
                targetDistance = Math.max(1.5, avgDistance + 1.0);
            }
        }

        mFocalX = 0.9f * mFocalX + 0.1f * (float) targetFocalX;
        mFocalY = 0.9f * mFocalY + 0.1f * (float) targetFocalY;
        mFocalZ = 0.9f * mFocalZ + 0.1f * (float) targetFocalZ;
        mVirtualDistance = 0.9f * mVirtualDistance + 0.1f * (float) targetDistance;

        if (mFrcMode) {
            // Draw FRC floor grid (zField = 0)
            for (double xVal = -8.5; xVal <= 8.5; xVal += 0.5) {
                float[] pStart = projectFieldPointToVirtual(xVal, -4.25, 0.0, width, height);
                float[] pEnd = projectFieldPointToVirtual(xVal, 4.25, 0.0, width, height);
                if (pStart != null && pEnd != null) {
                    canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], mGridPaint);
                }
            }
            for (double yVal = -4.25; yVal <= 4.25; yVal += 0.5) {
                float[] pStart = projectFieldPointToVirtual(-8.5, yVal, 0.0, width, height);
                float[] pEnd = projectFieldPointToVirtual(8.5, yVal, 0.0, width, height);
                if (pStart != null && pEnd != null) {
                    canvas.drawLine(pStart[0], pStart[1], pEnd[0], pEnd[1], mGridPaint);
                }
            }
        } else {
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
        }

        // 2. Draw camera axes and frustum
        float[] origin = null;
        float[] xAxisCam = null;
        float[] yAxisCam = null;
        float[] zAxisCam = null;
        float[] pf0 = null;
        float[] pf1 = null;
        float[] pf2 = null;
        float[] pf3 = null;

        if (mFrcMode) {
            if (cameraEstimated) {
                origin = projectCameraPointInFRC(0, 0, 0, camX_field, camY_field, camZ_field, R_f_c, width, height);
                xAxisCam = projectCameraPointInFRC(0.25, 0, 0, camX_field, camY_field, camZ_field, R_f_c, width, height);
                yAxisCam = projectCameraPointInFRC(0, 0.25, 0, camX_field, camY_field, camZ_field, R_f_c, width, height);
                zAxisCam = projectCameraPointInFRC(0, 0, 0.25, camX_field, camY_field, camZ_field, R_f_c, width, height);
                pf0 = projectCameraPointInFRC(-0.18, -0.13, 0.35, camX_field, camY_field, camZ_field, R_f_c, width, height);
                pf1 = projectCameraPointInFRC(0.18, -0.13, 0.35, camX_field, camY_field, camZ_field, R_f_c, width, height);
                pf2 = projectCameraPointInFRC(0.18, 0.13, 0.35, camX_field, camY_field, camZ_field, R_f_c, width, height);
                pf3 = projectCameraPointInFRC(-0.18, 0.13, 0.35, camX_field, camY_field, camZ_field, R_f_c, width, height);
            }
        } else {
            origin = projectPhysical3DToVirtual(0, 0, 0, width, height);
            xAxisCam = projectPhysical3DToVirtual(0.25, 0, 0, width, height);
            yAxisCam = projectPhysical3DToVirtual(0, 0.25, 0, width, height);
            zAxisCam = projectPhysical3DToVirtual(0, 0, 0.25, width, height);
            pf0 = projectPhysical3DToVirtual(-0.18, -0.13, 0.35, width, height);
            pf1 = projectPhysical3DToVirtual(0.18, -0.13, 0.35, width, height);
            pf2 = projectPhysical3DToVirtual(0.18, 0.13, 0.35, width, height);
            pf3 = projectPhysical3DToVirtual(-0.18, 0.13, 0.35, width, height);
        }

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

        // 3. Draw tags
        if (mFrcMode) {
            double s2 = 0.1651 / 2.0;
            for (int tagId = 1; tagId <= 32; tagId++) {
                double[][] T_field_tag = FRCTagLayout.TAG_TRANSFORMS[tagId];
                if (T_field_tag == null) continue;

                boolean isDetected = false;
                for (Pose3D pose : m3dPoses) {
                    if (pose.id == tagId) {
                        isDetected = true;
                        break;
                    }
                }

                double[][] corners_tag = {
                    {0.0, s2, s2},    // Top-Left (Corner 0)
                    {0.0, -s2, s2},   // Top-Right (Corner 1)
                    {0.0, -s2, -s2},  // Bottom-Right (Corner 2)
                    {0.0, s2, -s2}    // Bottom-Left (Corner 3)
                };

                float[][] corners_proj = new float[4][];
                boolean allCornersProjected = true;
                for (int j = 0; j < 4; j++) {
                    double x_f = T_field_tag[0][0]*corners_tag[j][0] + T_field_tag[0][1]*corners_tag[j][1] + T_field_tag[0][2]*corners_tag[j][2] + T_field_tag[0][3];
                    double y_f = T_field_tag[1][0]*corners_tag[j][0] + T_field_tag[1][1]*corners_tag[j][1] + T_field_tag[1][2]*corners_tag[j][2] + T_field_tag[1][3];
                    double z_f = T_field_tag[2][0]*corners_tag[j][0] + T_field_tag[2][1]*corners_tag[j][1] + T_field_tag[2][2]*corners_tag[j][2] + T_field_tag[2][3];
                    corners_proj[j] = projectFieldPointToVirtual(x_f, y_f, z_f, width, height);
                    if (corners_proj[j] == null) {
                        allCornersProjected = false;
                        break;
                    }
                }

                double tagX_f = T_field_tag[0][3];
                double tagY_f = T_field_tag[1][3];
                double tagZ_f = T_field_tag[2][3];
                float[] tagCenterProj = projectFieldPointToVirtual(tagX_f, tagY_f, tagZ_f, width, height);

                if (isDetected && origin != null && tagCenterProj != null) {
                    canvas.drawLine(origin[0], origin[1], tagCenterProj[0], tagCenterProj[1], mRayPaint);
                }

                if (allCornersProjected) {
                    Paint fillPaintToUse = mFacePaint;
                    Paint borderPaintToUse = mEdgePaint;
                    if (isDetected) {
                        borderPaintToUse = mTagAxisPaint;
                    } else {
                        borderPaintToUse = mGridPaint;
                    }

                    mPath.reset();
                    mPath.moveTo(corners_proj[0][0], corners_proj[0][1]);
                    mPath.lineTo(corners_proj[1][0], corners_proj[1][1]);
                    mPath.lineTo(corners_proj[2][0], corners_proj[2][1]);
                    mPath.lineTo(corners_proj[3][0], corners_proj[3][1]);
                    mPath.close();

                    canvas.drawPath(mPath, fillPaintToUse);
                    canvas.drawPath(mPath, borderPaintToUse);
                }

                // Draw coordinate axes on the tag center
                if (tagCenterProj != null) {
                    double L = tagSize;
                    double xAxisX_f = T_field_tag[0][0] * L + T_field_tag[0][3];
                    double xAxisY_f = T_field_tag[1][0] * L + T_field_tag[1][3];
                    double xAxisZ_f = T_field_tag[2][0] * L + T_field_tag[2][3];

                    double yAxisX_f = T_field_tag[0][1] * L + T_field_tag[0][3];
                    double yAxisY_f = T_field_tag[1][1] * L + T_field_tag[1][3];
                    double yAxisZ_f = T_field_tag[2][1] * L + T_field_tag[2][3];

                    double zAxisX_f = T_field_tag[0][2] * L + T_field_tag[0][3];
                    double zAxisY_f = T_field_tag[1][2] * L + T_field_tag[1][3];
                    double zAxisZ_f = T_field_tag[2][2] * L + T_field_tag[2][3];

                    float[] pxAxisProj = projectFieldPointToVirtual(xAxisX_f, xAxisY_f, xAxisZ_f, width, height);
                    float[] pyAxisProj = projectFieldPointToVirtual(yAxisX_f, yAxisY_f, yAxisZ_f, width, height);
                    float[] pzAxisProj = projectFieldPointToVirtual(zAxisX_f, zAxisY_f, zAxisZ_f, width, height);

                    if (pxAxisProj != null) {
                        mTagAxisPaint.setColor(0xFFFF3F3F); // Red (X)
                        canvas.drawLine(tagCenterProj[0], tagCenterProj[1], pxAxisProj[0], pxAxisProj[1], mTagAxisPaint);
                    }
                    if (pyAxisProj != null) {
                        mTagAxisPaint.setColor(0xFF3FDF3F); // Green (Y)
                        canvas.drawLine(tagCenterProj[0], tagCenterProj[1], pyAxisProj[0], pyAxisProj[1], mTagAxisPaint);
                    }
                    if (pzAxisProj != null) {
                        mTagAxisPaint.setColor(0xFF3F3FFF); // Blue (Z)
                        canvas.drawLine(tagCenterProj[0], tagCenterProj[1], pzAxisProj[0], pzAxisProj[1], mTagAxisPaint);
                    }
                }

                if (tagCenterProj != null) {
                    mTextPaint.setTextSize(24);
                    if (isDetected) {
                        mTextPaint.setColor(0xFF39FF14);
                    } else {
                        mTextPaint.setColor(0xFFCCCCCC);
                    }
                    String idStr = String.valueOf(tagId);
                    float textWidth = mTextPaint.measureText(idStr);
                    canvas.drawText(idStr, tagCenterProj[0] - textWidth/2f, tagCenterProj[1] + 8, mTextPaint);
                }
            }
        } else {
            // Draw detected tags (axes, wireframe cubes, tracking rays, labels)
            for (Pose3D pose : m3dPoses) {
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
        }

        // 4. Update diagnostics card telemetry
        if (mPoseTextView != null) {
            if (closestPose != null) {
                final String telemetry;
                if (mFrcMode) {
                    if (multiTagResult != null) {
                        FRCTagLayout.CameraPose cameraPose = multiTagResult.cameraPose;
                        telemetry = String.format("Target ID: %d (FRC Field Multi-Tag)\n" +
                                        "Field Pose: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                        "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                                multiTagResult.closestTagId, cameraPose.x, cameraPose.y, cameraPose.z,
                                cameraPose.roll, cameraPose.pitch, cameraPose.yaw);
                    } else {
                        telemetry = String.format("Target ID: %d (Invalid FRC Tag)\n" +
                                        "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                        "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                                closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                                closestPose.roll, closestPose.pitch, closestPose.yaw);
                    }
                } else {
                    telemetry = String.format("Target ID: %d\n" +
                                    "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                    "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                            closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                            closestPose.roll, closestPose.pitch, closestPose.yaw);
                }
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
            if (mRenderMode == MODE_3D) {
                render3DView(detections, canvas);
                return;
            } else if (mRenderMode == MODE_FRC) {
                renderFRCView(detections, canvas);
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

            mDefaultPoses.clear();
            for (ApriltagDetection detection : detections) {
                Pose3D pose = renderDetection(detection, canvas);
                if (pose != null) {
                    mDefaultPoses.add(pose);
                }

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
                    final String telemetry;
                    if (mFrcMode) {
                        MultiTagResult multiTagResult = computeMultiTagCameraPose(mDefaultPoses);
                        if (multiTagResult != null) {
                            FRCTagLayout.CameraPose cameraPose = multiTagResult.cameraPose;
                            telemetry = String.format("Target ID: %d (FRC Field Multi-Tag)\n" +
                                            "Field Pose: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                            "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                                    multiTagResult.closestTagId, cameraPose.x, cameraPose.y, cameraPose.z,
                                    cameraPose.roll, cameraPose.pitch, cameraPose.yaw);
                        } else {
                            telemetry = String.format("Target ID: %d (Invalid FRC Tag)\n" +
                                            "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                            "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                                    closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                                    closestPose.roll, closestPose.pitch, closestPose.yaw);
                        }
                    } else {
                        telemetry = String.format("Target ID: %d\n" +
                                        "Translation: X: %.3fm | Y: %.3fm | Z: %.3fm\n" +
                                        "Rotation: R: %.1f° | P: %.1f° | Y: %.1f°",
                                closestPose.id, closestPose.tx, closestPose.ty, closestPose.tz,
                                closestPose.roll, closestPose.pitch, closestPose.yaw);
                    }
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

            ArrayList<ApriltagDetection> detections = null;
            if (mCameraSize != null) {
                int fullW = mCameraSize.width;
                int fullH = mCameraSize.height;

                if (mRoiActive) {
                    int cropW = mRoiHalfSize * 2;
                    int cropH = mRoiHalfSize * 2;
                    int cropX = mRoiCenterX - mRoiHalfSize;
                    int cropY = mRoiCenterY - mRoiHalfSize;

                    // Boundary clipping
                    if (cropX < 0) cropX = 0;
                    if (cropY < 0) cropY = 0;
                    if (cropX + cropW > fullW) cropX = fullW - cropW;
                    if (cropY + cropH > fullH) cropY = fullH - cropH;

                    if (cropW <= fullW && cropH <= fullH && cropX >= 0 && cropY >= 0) {
                        try {
                            byte[] croppedY = cropYPlane(data, fullW, fullH, cropX, cropY, cropW, cropH);
                            detections = ApriltagNative.apriltag_detect_yuv_flat_decoded(croppedY, cropW, cropH);
                            
                            // Offset coordinates back to full image space
                            for (ApriltagDetection det : detections) {
                                det.c[0] += cropX;
                                det.c[1] += cropY;
                                for (int k = 0; k < 8; k += 2) {
                                    det.p[k] += cropX;
                                    det.p[k + 1] += cropY;
                                }
                            }
                        } catch (Exception e) {
                            Log.e(TAG, "Error during ROI detection: " + e.getMessage());
                            mRoiActive = false;
                        }
                    } else {
                        mRoiActive = false;
                    }
                }

                // Fallback to full frame detection if ROI is inactive or returned empty
                if (detections == null || (mRoiActive && detections.isEmpty())) {
                    detections = processCameraFrame(data, mCameraSize);
                    mRoiActive = false;
                }

                // Update tracking locks for the next frame
                if (detections.size() > 0) {
                    ApriltagDetection target = null;
                    if (mRoiActive) {
                        for (ApriltagDetection det : detections) {
                            if (det.id == mRoiLastTagId) {
                                target = det;
                                break;
                            }
                        }
                    }
                    if (target == null) {
                        // Target closest tag to screen center
                        double minDistance = Double.MAX_VALUE;
                        double centerX = fullW / 2.0;
                        double centerY = fullH / 2.0;
                        for (ApriltagDetection det : detections) {
                            double dist = Math.hypot(det.c[0] - centerX, det.c[1] - centerY);
                            if (dist < minDistance) {
                                minDistance = dist;
                                target = det;
                            }
                        }
                    }

                    if (target != null) {
                        mRoiActive = true;
                        mRoiLastTagId = target.id;
                        mRoiCenterX = (int) target.c[0];
                        mRoiCenterY = (int) target.c[1];

                        // Calculate adaptive ROI size based on tag dimensions in pixels
                        double w = Math.abs(target.p[2] - target.p[0]);
                        double h = Math.abs(target.p[5] - target.p[1]);
                        double tagSizeInPixels = Math.max(w, h);

                        mRoiHalfSize = (int) (tagSizeInPixels * 1.5);
                        if (mRoiHalfSize < 80) mRoiHalfSize = 80;
                        if (mRoiHalfSize > 240) mRoiHalfSize = 240;
                    } else {
                        mRoiActive = false;
                    }
                } else {
                    mRoiActive = false;
                }
            } else {
                detections = new ArrayList<>();
            }

            renderDetections(detections);

            BufferReleaseListener listener = mBufferReleaseListener;
            if (listener != null) {
                listener.onBufferReleased(data);
            }

            mLastDetectLatency = (System.currentTimeMillis() - mLastEnqueueFrameTime);
        }
    }
}