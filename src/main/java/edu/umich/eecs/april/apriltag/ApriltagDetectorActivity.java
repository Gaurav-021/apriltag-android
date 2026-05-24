package edu.umich.eecs.april.apriltag;

import android.Manifest;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.os.Bundle;
import android.preference.PreferenceManager;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import android.util.Log;
import android.util.TypedValue;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.TextView;
import android.view.MotionEvent;
import android.os.Build;
import android.view.Window;
import android.view.ViewGroup;
import androidx.core.view.ViewCompat;
import androidx.core.view.OnApplyWindowInsetsListener;
import androidx.core.view.WindowInsetsCompat;
import androidx.camera.view.PreviewView;


/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */

public class ApriltagDetectorActivity extends AppCompatActivity {
    private static final String TAG = "AprilTag";
    private DetectionThread mDetectionThread;
    private CameraXManager mCameraXManager;
    private int mRenderMode = DetectionThread.MODE_2D;

    private static final int MY_PERMISSIONS_REQUEST_CAMERA = 77;
    private int has_camera_permissions = 0;

    private void verifyPreferences() {
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);

        int nthreads = Integer.parseInt(sharedPreferences.getString("nthreads_value", "0"));
        if (nthreads <= 0) {
            int nproc = Runtime.getRuntime().availableProcessors();
            if (nproc <= 0) {
                nproc = 1;
            }
            Log.i(TAG, "available processors: " + nproc);
            PreferenceManager.getDefaultSharedPreferences(this).edit().putString("nthreads_value", Integer.toString(nproc)).apply();
        }

        if (!sharedPreferences.contains("apriltag_size")) {
            sharedPreferences.edit().putString("apriltag_size", "0.165").apply();
        }

        if (!sharedPreferences.contains("calibration_override")) {
            sharedPreferences.edit().putBoolean("calibration_override", false).apply();
        }
        if (!sharedPreferences.contains("calibration_fx")) {
            sharedPreferences.edit().putString("calibration_fx", "600.0").apply();
        }
        if (!sharedPreferences.contains("calibration_fy")) {
            sharedPreferences.edit().putString("calibration_fy", "600.0").apply();
        }
        if (!sharedPreferences.contains("calibration_cx")) {
            sharedPreferences.edit().putString("calibration_cx", "0.0").apply();
        }
        if (!sharedPreferences.contains("calibration_cy")) {
            sharedPreferences.edit().putString("calibration_cy", "0.0").apply();
        }
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Enable edge-to-edge drawing on API 21+ devices
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            Window window = getWindow();
            window.clearFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS
                    | WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION);
            window.getDecorView().setSystemUiVisibility(
                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                    | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                    | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
            );
            window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
            window.setStatusBarColor(Color.TRANSPARENT);
            window.setNavigationBarColor(Color.TRANSPARENT);
        }

        setContentView(R.layout.main);

        // Add toolbar/actionbar
        Toolbar myToolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(myToolbar);

        // Adjust views for status/navigation bar insets dynamically
        final View floatingHeader = findViewById(R.id.floatingHeaderContainer);
        if (floatingHeader != null) {
            ViewCompat.setOnApplyWindowInsetsListener(floatingHeader, new OnApplyWindowInsetsListener() {
                @Override
                public WindowInsetsCompat onApplyWindowInsets(View v, WindowInsetsCompat insets) {
                    ViewGroup.MarginLayoutParams lp = (ViewGroup.MarginLayoutParams) v.getLayoutParams();
                    // Shift toolbar down by status bar inset
                    lp.topMargin = insets.getSystemWindowInsetTop() + (int) (16 * getResources().getDisplayMetrics().density);
                    lp.leftMargin = insets.getSystemWindowInsetLeft() + (int) (16 * getResources().getDisplayMetrics().density);
                    lp.rightMargin = insets.getSystemWindowInsetRight() + (int) (16 * getResources().getDisplayMetrics().density);
                    v.setLayoutParams(lp);
                    return insets;
                }
            });
        }

        final View diagnosticsCard = findViewById(R.id.diagnosticsCard);
        if (diagnosticsCard != null) {
            ViewCompat.setOnApplyWindowInsetsListener(diagnosticsCard, new OnApplyWindowInsetsListener() {
                @Override
                public WindowInsetsCompat onApplyWindowInsets(View v, WindowInsetsCompat insets) {
                    ViewGroup.MarginLayoutParams lp = (ViewGroup.MarginLayoutParams) v.getLayoutParams();
                    // Shift card up/in by navigation bar / system gesture insets
                    lp.bottomMargin = insets.getSystemWindowInsetBottom() + (int) (16 * getResources().getDisplayMetrics().density);
                    lp.leftMargin = insets.getSystemWindowInsetLeft() + (int) (16 * getResources().getDisplayMetrics().density);
                    v.setLayoutParams(lp);
                    return insets;
                }
            });
        }

        // Make the screen stay awake
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // Ensure we have permission to use the camera (Permission Requesting for Android 6.0/SDK 23 and higher)
        if (ContextCompat.checkSelfPermission(this,
                Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            // Assume user knows enough about the app to know why we need the camera, just ask for permission
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.CAMERA},
                    MY_PERMISSIONS_REQUEST_CAMERA);
        } else {
            this.has_camera_permissions = 1;
        }
    }

    /** Release the camera when the application is exited */
    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopThreads();
        Log.i(TAG, "Finished destroying.");
    }

    /** Release the camera when application focus is lost */
    protected void onPause() {
        super.onPause();
        stopThreads();
        Log.i(TAG, "Finished pause.");
    }

    private void stopThreads() {
        if (mCameraXManager != null) {
            mCameraXManager.stop();
            mCameraXManager = null;
        }
        if (mDetectionThread != null) {
            mDetectionThread.interrupt();
            mDetectionThread.destroy();
            try {
                mDetectionThread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mDetectionThread = null;
        }
    }

    /** (Re-)initialize the camera */
    protected void onResume() {
        super.onResume();

        // Check permissions
        if (this.has_camera_permissions == 0) {
            Log.w(TAG, "Missing camera permissions.");
            return;
        }

        // DETECTION INIT
        // Re-initialize the Apriltag detector as settings may have changed
        verifyPreferences();
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);
        double decimation = Double.parseDouble(sharedPreferences.getString("decimation_list", "8"));
        double sigma = Double.parseDouble(sharedPreferences.getString("sigma_value", "0"));
        int nthreads = Integer.parseInt(sharedPreferences.getString("nthreads_value", "4"));
        int max_hamming_error = Integer.parseInt(sharedPreferences.getString("max_hamming_error", "0"));
        boolean diagnosticsEnabled = sharedPreferences.getBoolean("diagnostics_enabled", true);
        String tagFamily = sharedPreferences.getString("tag_family_list", "tag36h11");
        Log.i(TAG, String.format("decimation: %f | sigma: %f | nthreads: %d | tagFamily: %s",
                decimation, sigma, nthreads, tagFamily));
        ApriltagNative.apriltag_init(tagFamily, max_hamming_error, decimation, sigma, nthreads);

        // DIAGNOSTICS
        findViewById(R.id.detectionFpsTextView).setVisibility(diagnosticsEnabled ? View.VISIBLE : View.INVISIBLE);
        findViewById(R.id.previewFpsTextView).setVisibility(diagnosticsEnabled ? View.VISIBLE : View.INVISIBLE);
        TextView poseTextView = (TextView) findViewById(R.id.poseTextView);
        if (poseTextView != null) {
            poseTextView.setVisibility(diagnosticsEnabled ? View.VISIBLE : View.GONE);
            poseTextView.setTypeface(poseTextView.getTypeface(), Typeface.BOLD);
        }
        TextView tagFamilyText = (TextView) findViewById(R.id.tagFamily);
        stylizeText(tagFamilyText);
        tagFamilyText.setText("Tag Family: " + tagFamily.substring(3));

        // THREAD INIT
        // Start the detection process on a separate thread
        TextureView detectionSurface = (TextureView) findViewById(R.id.tagView);
        TextView detectionFpsTextView = (TextView) findViewById(R.id.detectionFpsTextView);
        stylizeText(detectionFpsTextView);
        TextView poseTextVal = (TextView) findViewById(R.id.poseTextView);
        mDetectionThread = new DetectionThread(detectionSurface, detectionFpsTextView, poseTextVal);
        mDetectionThread.setRenderMode(mRenderMode);
        mDetectionThread.initialize();
        mDetectionThread.start();

        detectionSurface.setOnTouchListener(new View.OnTouchListener() {
            private float lastX;
            private float lastY;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (mRenderMode != DetectionThread.MODE_3D) {
                    return false;
                }
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        lastX = event.getX();
                        lastY = event.getY();
                        return true;
                    case MotionEvent.ACTION_MOVE:
                        float dx = event.getX() - lastX;
                        float dy = event.getY() - lastY;
                        lastX = event.getX();
                        lastY = event.getY();

                        if (mDetectionThread != null) {
                            float scale = 0.005f;
                            mDetectionThread.updateVirtualCameraOrbit(-dx * scale, -dy * scale);
                        }
                        return true;
                }
                return false;
            }
        });

        // Start the CameraX preview and stream analyzer
        PreviewView previewView = findViewById(R.id.previewView);
        TextView previewFpsTextView = findViewById(R.id.previewFpsTextView);
        stylizeText(previewFpsTextView);
        mCameraXManager = new CameraXManager(this, previewView, mDetectionThread, previewFpsTextView);
        mCameraXManager.start();
    }

    private void stylizeText(TextView textView) {
        if (textView.getId() == R.id.tagFamily) {
            textView.setTextColor(ContextCompat.getColor(this, R.color.umMaize));
            textView.setTextSize(TypedValue.COMPLEX_UNIT_SP, 14);
        } else {
            textView.setTextColor(Color.WHITE);
            textView.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        }
        textView.setTypeface(textView.getTypeface(), Typeface.BOLD);
    }
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        MenuItem item = menu.findItem(R.id.toggle_3d);
        if (item != null) {
            updateMenuTitle(item);
        }
        return true;
    }

    private void updateMenuTitle(MenuItem item) {
        if (mRenderMode == DetectionThread.MODE_2D) {
            item.setTitle(R.string.toggle_2d);
        } else if (mRenderMode == DetectionThread.MODE_3D) {
            item.setTitle(R.string.toggle_3d);
        } else {
            item.setTitle(R.string.toggle_frc);
        }
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();

        if (id == R.id.toggle_3d) {
            SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);
            boolean frcMode = sharedPreferences.getBoolean("frc_mode", false);
            if (frcMode) {
                if (mRenderMode == DetectionThread.MODE_2D) {
                    mRenderMode = DetectionThread.MODE_3D;
                } else if (mRenderMode == DetectionThread.MODE_3D) {
                    mRenderMode = DetectionThread.MODE_FRC;
                } else {
                    mRenderMode = DetectionThread.MODE_2D;
                }
            } else {
                if (mRenderMode == DetectionThread.MODE_2D) {
                    mRenderMode = DetectionThread.MODE_3D;
                } else {
                    mRenderMode = DetectionThread.MODE_2D;
                }
            }
            updateMenuTitle(item);
            if (mDetectionThread != null) {
                mDetectionThread.setRenderMode(mRenderMode);
            }
            return true;
        } else if (id == R.id.settings) {
            verifyPreferences();
            Intent intent = new Intent();
            intent.setClassName(this, "edu.umich.eecs.april.apriltag.SettingsActivity");
            startActivity(intent);
            return true;
        } else if (id == R.id.reset) {
            // Reset all shared preferences to default values
            PreferenceManager.getDefaultSharedPreferences(this).edit().clear().commit();

            // Restart the camera preview
            onPause();
            onResume();

            return true;
        } else {
            return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_CAMERA: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    Log.i(TAG, "App GRANTED camera permissions");

                    // Set flag
                    this.has_camera_permissions = 1;

                    // Restart the camera
                    onPause();
                    onResume();
                } else {
                    Log.i(TAG, "App DENIED camera permissions");
                    this.has_camera_permissions = 0;
                }
                return;
            }
        }
    }
}
