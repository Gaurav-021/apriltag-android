package edu.umich.eecs.april.apriltag;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.preference.PreferenceManager;
import android.util.Log;
import android.util.Size;
import android.util.SizeF;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.camera2.interop.Camera2CameraInfo;
import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.google.common.util.concurrent.ListenableFuture;

import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class CameraXManager {
    private static final String TAG = "CameraXManager";

    private final AppCompatActivity mActivity;
    private final PreviewView mPreviewView;
    private final DetectionThread mDetectionThread;
    private final TextView mFpsTextView;

    private ProcessCameraProvider mCameraProvider;
    private ExecutorService mCameraExecutor;

    private long mLastRender = System.currentTimeMillis();
    private int mFrameCount = 0;

    public CameraXManager(AppCompatActivity activity, PreviewView previewView, DetectionThread detectionThread, TextView fpsTextView) {
        mActivity = activity;
        mPreviewView = previewView;
        mDetectionThread = detectionThread;
        mFpsTextView = fpsTextView;
        mCameraExecutor = Executors.newSingleThreadExecutor();
    }

    public void start() {
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(mActivity);
        cameraProviderFuture.addListener(() -> {
            try {
                mCameraProvider = cameraProviderFuture.get();
                bindCameraUseCases();
            } catch (Exception e) {
                Log.e(TAG, "Error obtaining ProcessCameraProvider: " + e.getMessage());
            }
        }, ContextCompat.getMainExecutor(mActivity));
    }

    private void bindCameraUseCases() {
        if (mCameraProvider == null) return;

        mCameraProvider.unbindAll();

        // 1. Choose Lens Facing
        CameraSelector cameraSelector = new CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .build();

        // 2. Load and Resolve Resolution
        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(mActivity);
        String resSetting = sharedPref.getString("preview_resolution", "largest");
        
        Size targetSize = null;
        if ("largest".equals(resSetting)) {
            targetSize = getLargestSupportedSize(mActivity);
        } else {
            String[] parts = resSetting.split("x");
            if (parts.length == 2) {
                targetSize = new Size(Integer.parseInt(parts[0]), Integer.parseInt(parts[1]));
            }
        }

        if (targetSize == null) {
            targetSize = new Size(1280, 720); // standard default
        }

        Log.i(TAG, "Binding CameraX with target resolution: " + targetSize.getWidth() + "x" + targetSize.getHeight());

        // 3. Configure Preview Use Case
        Preview.Builder previewBuilder = new Preview.Builder();
        previewBuilder.setTargetResolution(targetSize);
        Preview preview = previewBuilder.build();
        preview.setSurfaceProvider(mPreviewView.getSurfaceProvider());

        // 4. Configure Image Analysis Use Case
        ImageAnalysis.Builder analysisBuilder = new ImageAnalysis.Builder();
        analysisBuilder.setTargetResolution(targetSize);
        analysisBuilder.setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST);
        analysisBuilder.setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888);
        ImageAnalysis imageAnalysis = analysisBuilder.build();

        imageAnalysis.setAnalyzer(mCameraExecutor, new ImageAnalysis.Analyzer() {
            @Override
            public void analyze(@NonNull ImageProxy image) {
                try {
                    int width = image.getWidth();
                    int height = image.getHeight();

                    // Y plane is the first plane
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

                    mDetectionThread.enqueueCameraFrame(yBytes, width, height);

                    previewFpsCallback();
                } catch (Exception e) {
                    Log.e(TAG, "Error enqueuing analyzer frame: " + e.getMessage());
                } finally {
                    image.close();
                }
            }
        });

        // 5. Bind use cases to Lifecycle
        try {
            Camera camera = mCameraProvider.bindToLifecycle(
                    (LifecycleOwner) mActivity,
                    cameraSelector,
                    preview,
                    imageAnalysis
            );

            // Fetch Camera FOV dynamically via Camera2 CameraCharacteristics interop
            CameraCharacteristics characteristics = Camera2CameraInfo.extractCameraCharacteristics(camera.getCameraInfo());
            float[] focalLengths = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
            SizeF sensorSize = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
            if (focalLengths != null && focalLengths.length > 0 && sensorSize != null) {
                float focalLength = focalLengths[0];
                float sensorW = sensorSize.getWidth();
                float sensorH = sensorSize.getHeight();
                float fovH = (float) Math.toDegrees(2 * Math.atan(sensorW / (2 * focalLength)));
                float fovV = (float) Math.toDegrees(2 * Math.atan(sensorH / (2 * focalLength)));
                mDetectionThread.setCameraFov(fovH, fovV);
                Log.i(TAG, "Resolved FOV dynamically: h=" + fovH + ", v=" + fovV);
            } else {
                mDetectionThread.setCameraFov(60.0f, 45.0f);
            }
        } catch (Exception e) {
            Log.e(TAG, "Use case binding failed: " + e.getMessage());
        }
    }

    private Size getLargestSupportedSize(Context context) {
        try {
            CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            for (String id : manager.getCameraIdList()) {
                CameraCharacteristics characteristics = manager.getCameraCharacteristics(id);
                Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                    StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                    if (map != null) {
                        Size[] sizes = map.getOutputSizes(SurfaceTexture.class);
                        if (sizes != null && sizes.length > 0) {
                            Size largest = sizes[0];
                            for (Size s : sizes) {
                                if (s.getWidth() * s.getHeight() > largest.getWidth() * largest.getHeight()) {
                                    largest = s;
                                }
                            }
                            return largest;
                        }
                    }
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "Failed to resolve largest preview size: " + e.getMessage());
        }
        return null;
    }

    private void previewFpsCallback() {
        long now = System.currentTimeMillis();
        long diff = now - mLastRender;
        mFrameCount++;
        if (diff >= 1000) {
            final double fps = 1000.0 / diff * mFrameCount;
            mFpsTextView.post(() -> mFpsTextView.setText(String.format("%.2f fps Camera", fps)));
            mLastRender = now;
            mFrameCount = 0;
        }
    }

    public void stop() {
        if (mCameraProvider != null) {
            mCameraProvider.unbindAll();
        }
        if (mCameraExecutor != null) {
            mCameraExecutor.shutdown();
            mCameraExecutor = null;
        }
    }
}
