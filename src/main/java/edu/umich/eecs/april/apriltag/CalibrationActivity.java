package edu.umich.eecs.april.apriltag;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.util.Size;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.google.common.util.concurrent.ListenableFuture;

import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class CalibrationActivity extends AppCompatActivity {
    private static final String TAG = "CalibrationActivity";
    private static final int CAMERA_PERMISSION_REQUEST = 101;
    
    // 9 columns and 6 rows of inner corners (standard 10x7 chessboard)
    private static final int PATTERN_COLS = 9;
    private static final int PATTERN_ROWS = 6;
    private static final int REQUIRED_FRAMES = 12;

    private PreviewView mPreviewView;
    private CalibrationOverlayView mOverlayView;
    private TextView mProgressText;
    private TextView mStatusText;
    private Button mBtnCalibrate;
    
    private ProcessCameraProvider mCameraProvider;
    private ExecutorService mCameraExecutor;
    
    // Calibration variables
    private final Size mPatternSize = new Size(PATTERN_COLS, PATTERN_ROWS);
    private final List<Mat> mImagePointsList = new ArrayList<>();
    private final List<Mat> mObjectPointsList = new ArrayList<>();
    private MatOfPoint3f mSingleObjectPoints;
    
    private boolean mIsCaptureRequested = false;
    private MatOfPoint2f mLastDetectedCorners = null;
    private int mFrameWidth = 0;
    private int mFrameHeight = 0;
    
    // Auto-capture variables
    private float[] mLastCornersArray = null;
    private long mLastStableTime = 0;
    private static final double STABILITY_THRESHOLD_PX = 10.0;
    private static final long STABILITY_DURATION_MS = 800;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calibration);
        
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            Log.e(TAG, "OpenCV initialization failed!");
            Toast.makeText(this, "OpenCV initialization failed!", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        mPreviewView = findViewById(R.id.calibrationPreview);
        mOverlayView = findViewById(R.id.calibrationOverlay);
        mProgressText = findViewById(R.id.calibrationProgressText);
        mStatusText = findViewById(R.id.calibrationStatusText);
        
        Button btnCancel = findViewById(R.id.btnCancelCalibration);
        btnCancel.setOnClickListener(v -> finish());
        
        View btnCapture = findViewById(R.id.btnCaptureFrame);
        btnCapture.setOnClickListener(v -> triggerCapture());
        
        mBtnCalibrate = findViewById(R.id.btnRunCalibration);
        mBtnCalibrate.setOnClickListener(v -> runCalibrationSolver());
        
        mCameraExecutor = Executors.newSingleThreadExecutor();
        
        // Prepare static 3D object points
        mSingleObjectPoints = new MatOfPoint3f();
        Point3[] points3D = new Point3[PATTERN_COLS * PATTERN_ROWS];
        for (int r = 0; r < PATTERN_ROWS; r++) {
            for (int c = 0; c < PATTERN_COLS; c++) {
                points3D[r * PATTERN_COLS + c] = new Point3(c, r, 0.0);
            }
        }
        mSingleObjectPoints.fromArray(points3D);

        // Check Permissions
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{android.Manifest.permission.CAMERA}, CAMERA_PERMISSION_REQUEST);
        } else {
            startCamera();
        }
    }

    private void startCamera() {
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                mCameraProvider = cameraProviderFuture.get();
                bindUseCases();
            } catch (Exception e) {
                Log.e(TAG, "Error starting camera provider: " + e.getMessage());
            }
        }, ContextCompat.getMainExecutor(this));
    }

    private void bindUseCases() {
        if (mCameraProvider == null) return;
        mCameraProvider.unbindAll();

        CameraSelector cameraSelector = new CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .build();

        // Standard 720p or 1080p target resolution for calibration
        Size targetSize = new Size(1280, 720);

        Preview preview = new Preview.Builder()
                .setTargetResolution(targetSize)
                .build();
        preview.setSurfaceProvider(mPreviewView.getSurfaceProvider());

        ImageAnalysis imageAnalysis = new ImageAnalysis.Builder()
                .setTargetResolution(targetSize)
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build();

        imageAnalysis.setAnalyzer(mCameraExecutor, new ImageAnalysis.Analyzer() {
            @Override
            public void analyze(@NonNull ImageProxy image) {
                try {
                    int width = image.getWidth();
                    int height = image.getHeight();
                    mFrameWidth = width;
                    mFrameHeight = height;

                    ImageProxy.PlaneProxy yPlane = image.getPlanes()[0];
                    ByteBuffer yBuffer = yPlane.getBuffer();
                    int rowStride = yPlane.getRowStride();

                    byte[] yBytes = new byte[width * height];
                    if (rowStride == width) {
                        yBuffer.get(yBytes);
                    } else {
                        for (int row = 0; row < height; row++) {
                            yBuffer.position(row * rowStride);
                            yBuffer.get(yBytes, row * width, width);
                        }
                    }

                    // Process with OpenCV
                    Mat grayMat = new Mat(height, width, CvType.CV_8UC1);
                    grayMat.put(0, 0, yBytes);

                    MatOfPoint2f corners = new MatOfPoint2f();
                    org.opencv.core.Size patternSize = new org.opencv.core.Size(PATTERN_COLS, PATTERN_ROWS);
                    
                    boolean found = Calib3d.findChessboardCorners(
                            grayMat,
                            patternSize,
                            corners,
                            Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK
                    );

                    if (found) {
                        float[] cornersArray = new float[PATTERN_COLS * PATTERN_ROWS * 2];
                        Point[] points = corners.toArray();
                        for (int i = 0; i < points.length; i++) {
                            cornersArray[i * 2] = (float) points[i].x;
                            cornersArray[i * 2 + 1] = (float) points[i].y;
                        }

                        // Map corners to layout canvas (FILL_CENTER scale and offset)
                        float[] mappedCorners = mapCornersToCanvas(cornersArray, width, height);
                        mOverlayView.setCorners(mappedCorners, PATTERN_ROWS, PATTERN_COLS);

                        checkAutoCapture(cornersArray);

                        synchronized (CalibrationActivity.this) {
                            if (mIsCaptureRequested) {
                                mImagePointsList.add(corners.clone());
                                mObjectPointsList.add(mSingleObjectPoints.clone());
                                mIsCaptureRequested = false;
                                
                                runOnUiThread(() -> {
                                    updateProgressUI();
                                    Toast.makeText(CalibrationActivity.this, "Frame Captured!", Toast.LENGTH_SHORT).show();
                                });
                            }
                        }
                    } else {
                        mOverlayView.clearCorners();
                        mLastCornersArray = null;
                        mLastStableTime = 0;
                    }
                    
                    grayMat.release();
                } catch (Exception e) {
                    Log.e(TAG, "Error in calibration analyzer: " + e.getMessage());
                } finally {
                    image.close();
                }
            }
        });

        try {
            mCameraProvider.bindToLifecycle(
                    (LifecycleOwner) this,
                    cameraSelector,
                    preview,
                    imageAnalysis
            );
        } catch (Exception e) {
            Log.e(TAG, "Binding use cases failed: " + e.getMessage());
        }
    }

    private float[] mapCornersToCanvas(float[] corners, int frameW, int frameH) {
        float[] mapped = new float[corners.length];
        float viewW = mOverlayView.getWidth();
        float viewH = mOverlayView.getHeight();
        
        // Since frame is landscape but screen is portrait:
        // frameW = vertical coordinates, frameH = horizontal coordinates
        float scale = Math.max(viewW / frameH, viewH / frameW);
        float scaledW = frameH * scale;
        float scaledH = frameW * scale;
        
        float offsetX = (viewW - scaledW) / 2.0f;
        float offsetY = (viewH - scaledH) / 2.0f;

        for (int i = 0; i < corners.length / 2; i++) {
            float xImg = corners[i * 2];
            float yImg = corners[i * 2 + 1];
            // Rotate 90 degrees: x_screen = (frameH - y_img) * scale + offsetX
            mapped[i * 2] = (frameH - yImg) * scale + offsetX;
            // y_screen = x_img * scale + offsetY
            mapped[i * 2 + 1] = xImg * scale + offsetY;
        }
        return mapped;
    }

    private void checkAutoCapture(float[] corners) {
        if (mLastCornersArray == null) {
            mLastCornersArray = corners;
            mLastStableTime = System.currentTimeMillis();
            return;
        }

        // Calculate maximum distance shift between last corners
        double maxShift = 0.0;
        for (int i = 0; i < corners.length; i++) {
            double shift = Math.abs(corners[i] - mLastCornersArray[i]);
            if (shift > maxShift) {
                maxShift = shift;
            }
        }

        mLastCornersArray = corners;

        if (maxShift < STABILITY_THRESHOLD_PX) {
            long stableDuration = System.currentTimeMillis() - mLastStableTime;
            if (stableDuration > STABILITY_DURATION_MS) {
                // Auto capture!
                triggerCapture();
                mLastStableTime = System.currentTimeMillis() + 1000; // block auto-capture for 1 sec
            }
        } else {
            mLastStableTime = System.currentTimeMillis();
        }
    }

    private synchronized void triggerCapture() {
        if (mImagePointsList.size() >= REQUIRED_FRAMES) {
            runOnUiThread(() -> Toast.makeText(this, "Enough frames captured. Tap Calibrate!", Toast.LENGTH_SHORT).show());
            return;
        }
        mIsCaptureRequested = true;
    }

    private void updateProgressUI() {
        int count = mImagePointsList.size();
        mProgressText.setText("Captured Frames: " + count + " / " + REQUIRED_FRAMES);
        
        if (count >= REQUIRED_FRAMES) {
            mBtnCalibrate.setEnabled(true);
            mBtnCalibrate.setTextColor(0xFF00E5FF);
            mStatusText.setText("Ready to calibrate. Tap 'Calibrate' to calculate camera parameters.");
        } else {
            mBtnCalibrate.setEnabled(false);
            mBtnCalibrate.setTextColor(0xFF888888);
            mStatusText.setText("Hold checkerboard pattern still. Need " + (REQUIRED_FRAMES - count) + " more frames.");
        }
    }

    private void runCalibrationSolver() {
        if (mImagePointsList.size() < REQUIRED_FRAMES) return;

        mStatusText.setText("Calculating calibration matrix... Please wait.");
        mCameraExecutor.execute(() -> {
            try {
                Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
                Mat distCoeffs = new Mat();
                List<Mat> rvecs = new ArrayList<>();
                List<Mat> tvecs = new ArrayList<>();

                org.opencv.core.Size imageSize = new org.opencv.core.Size(mFrameWidth, mFrameHeight);
                double rms = Calib3d.calibrateCamera(
                        mObjectPointsList,
                        mImagePointsList,
                        imageSize,
                        cameraMatrix,
                        distCoeffs,
                        rvecs,
                        tvecs
                );

                double fx = cameraMatrix.get(0, 0)[0];
                double fy = cameraMatrix.get(1, 1)[0];
                double cx = cameraMatrix.get(0, 2)[0];
                double cy = cameraMatrix.get(1, 2)[0];

                runOnUiThread(() -> showResultsDialog(rms, fx, fy, cx, cy));
            } catch (Exception e) {
                Log.e(TAG, "Calibration calculation error: " + e.getMessage());
                runOnUiThread(() -> {
                    Toast.makeText(this, "Calibration Failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
                    updateProgressUI();
                });
            }
        });
    }

    private void showResultsDialog(double rms, double fx, double fy, double cx, double cy) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Calibration Completed");
        builder.setMessage(String.format("Root Mean Squared Error (RMS): %.4f pixels\n\n" +
                        "Intrinsics Matrix:\n" +
                        "fx: %.2f\n" +
                        "fy: %.2f\n" +
                        "cx: %.2f\n" +
                        "cy: %.2f\n\n" +
                        "Resolution: %d x %d\n\n" +
                        "Would you like to save these values as your override calibration?",
                rms, fx, fy, cx, cy, mFrameWidth, mFrameHeight));

        builder.setPositiveButton("Save", (dialog, which) -> {
            saveCalibrationToPrefs(fx, fy, cx, cy);
            Toast.makeText(this, "Calibration Saved!", Toast.LENGTH_SHORT).show();
            finish();
        });

        builder.setNegativeButton("Recalibrate", (dialog, which) -> {
            mImagePointsList.clear();
            mObjectPointsList.clear();
            updateProgressUI();
        });

        builder.setNeutralButton("Cancel", null);
        builder.show();
    }

    private void saveCalibrationToPrefs(double fx, double fy, double cx, double cy) {
        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor = sharedPref.edit();
        
        editor.putString("calibration_fx", String.valueOf(fx));
        editor.putString("calibration_fy", String.valueOf(fy));
        editor.putString("calibration_cx", String.valueOf(cx));
        editor.putString("calibration_cy", String.valueOf(cy));
        editor.putInt("calibration_width", mFrameWidth);
        editor.putInt("calibration_height", mFrameHeight);
        editor.putBoolean("calibration_override", true);
        editor.apply();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA_PERMISSION_REQUEST) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startCamera();
            } else {
                Toast.makeText(this, "Camera permission required for calibration!", Toast.LENGTH_LONG).show();
                finish();
            }
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mCameraProvider != null) {
            mCameraProvider.unbindAll();
        }
        if (mCameraExecutor != null) {
            mCameraExecutor.shutdown();
        }
        
        // Clean up matrices
        for (Mat m : mImagePointsList) m.release();
        for (Mat m : mObjectPointsList) m.release();
        if (mSingleObjectPoints != null) mSingleObjectPoints.release();
    }
}
