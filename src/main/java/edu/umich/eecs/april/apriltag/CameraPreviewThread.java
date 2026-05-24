package edu.umich.eecs.april.apriltag;

import android.hardware.Camera;
import android.util.Log;
import android.view.SurfaceHolder;
import android.widget.TextView;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**

 This class is responsible for managing the camera and live preview. It also enqueues image previews
 in the DetectionThread for asynchronous Apriltag detection on a separate view.
 <p>
 This class also displays a text view with the current frames per second (FPS) of the camera thread.
 </p>
 */
public class CameraPreviewThread extends Thread {
    private static final String TAG = "CameraPreviewThread";

    private final SurfaceHolder mSurfaceHolder;
    private final DetectionThread mDetectionThread;
    private Camera mCamera;
    private final TextView mFpsTextView;

    private long mLastRender = System.currentTimeMillis();
    private int mFrameCount = 0;
    private SurfaceHolder.Callback mCallback = new SurfaceHolder.Callback() {
        @Override
        public void surfaceCreated(SurfaceHolder holder) {
            try {
                mCamera.setPreviewDisplay(holder);

                // Set up buffer recycling callback listener
                mDetectionThread.setBufferReleaseListener(data -> {
                    if (mCamera != null) {
                        try {
                            mCamera.addCallbackBuffer(data);
                        } catch (Exception e) {
                            Log.e(TAG, "Error returning callback buffer: " + e.getMessage());
                        }
                    }
                });

                // Calculate required frame buffer size (NV21 format)
                Camera.Size previewSize = mCamera.getParameters().getPreviewSize();
                int bufferSize = previewSize.width * previewSize.height * 3 / 2;

                // Pre-allocate 3 buffers and add them to the camera preview callback queue
                for (int i = 0; i < 3; i++) {
                    mCamera.addCallbackBuffer(new byte[bufferSize]);
                }

                // Register preview callback with recycled buffers
                mCamera.setPreviewCallbackWithBuffer((data, camera) -> {
                    try {
                        mDetectionThread.enqueueCameraFrame(data, camera.getParameters().getPreviewSize());
                    } catch (InterruptedException e) {
                        Log.e(TAG, "Interrupted while enqueuing camera frame: " + e.getMessage());
                    }

                    previewFpsCallback();
                });

                mCamera.startPreview();
            } catch (IOException e) {
                Log.e(TAG, "Error setting camera preview: " + e.getMessage());
            }
        }

        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
            // Do nothing
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder holder) {
            // Do nothing
        }
    };

    public CameraPreviewThread(SurfaceHolder surfaceHolder, DetectionThread detectionThread, TextView fpsTextView) {
        mSurfaceHolder = surfaceHolder;
        mFpsTextView = fpsTextView;
        mDetectionThread = detectionThread;

        mSurfaceHolder.addCallback(mCallback);
    }

    public void destroy() {
        mSurfaceHolder.removeCallback(mCallback);
        mDetectionThread.setBufferReleaseListener(null);
        if (mCamera != null) {
            mCamera.setPreviewCallbackWithBuffer(null);
            mCamera.stopPreview();
            mCamera.release();
            mCamera = null;
        }
    }

    private void previewFpsCallback() {
        long now = System.currentTimeMillis();
        long diff = now - mLastRender;
        mFrameCount++;
        if (diff >= 1000) {
            double fps = 1000.0 / diff * mFrameCount;
            mFpsTextView.setText(String.format("%.2f fps Camera", fps));
            mLastRender = now;
            mFrameCount = 0;
        }
    }

    @Override
    public void run() {
        // Stop the previous camera preview
        if (mCamera != null) {
            try {
                mCamera.stopPreview();
                Log.i(TAG, "Camera stop");
            } catch (Exception e) {
                Log.e(TAG, "Unable to stop camera: " + e);
            }
        }

        try {
            mCamera.setPreviewDisplay(mSurfaceHolder);
            mCamera.startPreview();
            Log.i(TAG, "Camera preview start");
        } catch (IOException e) {
            Log.e(TAG, "Error setting camera preview: " + e.getMessage());
        }
    }

    protected void initialize() {
        int camidx = 0;
        Camera.CameraInfo info = new Camera.CameraInfo();
        for (int i = 0; i < Camera.getNumberOfCameras(); i += 1) {
            Camera.getCameraInfo(i, info);
            int desiredFacing = Camera.CameraInfo.CAMERA_FACING_BACK;
            if (info.facing == desiredFacing) {
                camidx = i;
                break;
            }
        }

        try {
            mCamera = Camera.open(camidx);
            Log.i(TAG, "using camera " + camidx);
        } catch (Exception e) {
            Log.e(TAG, "Couldn't open camera: " + e.getMessage());
            return;
        }

        Camera.getCameraInfo(camidx, info);
        setCameraParameters(mCamera, info);
    }

    private void setCameraParameters(Camera camera, Camera.CameraInfo info)
    {
        Camera.Parameters parameters = camera.getParameters();

        List<Camera.Size> sizeList = camera.getParameters().getSupportedPreviewSizes();
        
        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(mFpsTextView.getContext());
        String resSetting = sharedPref.getString("preview_resolution", "largest");
        
        Camera.Size bestSize = null;
        if ("largest".equals(resSetting)) {
            for (int i = 0; i < sizeList.size(); i++) {
                Camera.Size candidateSize = sizeList.get(i);
                Log.i(TAG, " " + candidateSize.width + "x" + candidateSize.height + " (" + candidateSize.width * candidateSize.height + " area)");
                if (bestSize == null || (candidateSize.width * candidateSize.height) > (bestSize.width * bestSize.height)) {
                    if (candidateSize.width != candidateSize.height) {
                        bestSize = candidateSize;
                    }
                }
            }
        } else {
            String[] parts = resSetting.split("x");
            if (parts.length == 2) {
                int targetW = Integer.parseInt(parts[0]);
                int targetH = Integer.parseInt(parts[1]);
                double minDiff = Double.MAX_VALUE;
                for (Camera.Size candidateSize : sizeList) {
                    if (candidateSize.width == candidateSize.height) continue;
                    double diff = Math.abs(candidateSize.width - targetW) + Math.abs(candidateSize.height - targetH);
                    if (diff < minDiff) {
                        bestSize = candidateSize;
                        minDiff = diff;
                    }
                }
            }
        }
        
        if (bestSize == null && sizeList.size() > 0) {
            bestSize = sizeList.get(0);
        }
        
        parameters.setPreviewSize(bestSize.width, bestSize.height);
        Log.i(TAG, "Setting preview size: " + bestSize.width + " x " + bestSize.height);

        List<String> focusModes = parameters.getSupportedFocusModes();
        if (focusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO)) {
            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
            Log.i(TAG, "Setting focus mode for continuous video");
        } else {
            Log.i(TAG, "Focus mode for continuous video not supported, skipping");
        }

        List<int[]> fpsRanges = parameters.getSupportedPreviewFpsRange();
        Log.i(TAG, "Supported FPS ranges:");
        int[] bestRange = null;
        for (int[] range : fpsRanges) {
            Log.i(TAG, "  [" + range[0] + ", " + range[1] + "]");
            if (bestRange == null) {
                bestRange = range;
            } else {
                // Prefer higher maximum frame rate. If maximums are equal, prefer higher minimum (more constant) frame rate.
                if (range[1] > bestRange[1]) {
                    bestRange = range;
                } else if (range[1] == bestRange[1] && range[0] > bestRange[0]) {
                    bestRange = range;
                }
            }
        }

        if (bestRange != null) {
            parameters.setPreviewFpsRange(bestRange[0], bestRange[1]);
            Log.i(TAG, "Setting FPS range [" + bestRange[0] + ", " + bestRange[1] + "]");
        }

        camera.setDisplayOrientation(info.orientation % 360);

        camera.setParameters(parameters);

        float fovH = parameters.getHorizontalViewAngle();
        float fovV = parameters.getVerticalViewAngle();
        mDetectionThread.setCameraFov(fovH, fovV);
    }
}