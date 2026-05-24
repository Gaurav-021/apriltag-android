package edu.umich.eecs.april.apriltag;


import android.annotation.TargetApi;
import android.app.ActionBar;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.res.Configuration;
import android.os.Build;
import android.os.Bundle;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceActivity;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.preference.SwitchPreference;
import android.util.Log;
import android.view.MenuItem;
import android.hardware.Camera;
import java.util.ArrayList;
import java.util.List;

/**
 * A {@link PreferenceActivity} that presents a set of application settings. On
 * handset devices, settings are presented as a single list. On tablets,
 * settings are split by category, with category headers shown to the left of
 * the list of settings.
 * <p/>
 * See <a href="http://developer.android.com/design/patterns/settings.html">
 * Android Design: Settings</a> for design guidelines and the <a
 * href="http://developer.android.com/guide/topics/ui/settings.html">Settings
 * API Guide</a> for more information on developing a Settings UI.
 */
public class SettingsActivity extends PreferenceActivity {
    /**
     * A preference value change listener that updates the preference's summary
     * to reflect its new value.
     */
    private static Preference.OnPreferenceChangeListener sBindPreferenceSummaryToValueListener = new Preference.OnPreferenceChangeListener() {
        @Override
        public boolean onPreferenceChange(Preference preference, Object value) {
            String stringValue = value.toString();

            if (preference instanceof ListPreference) {
                // For list preferences, look up the correct display value in
                // the preference's 'entries' list.
                ListPreference listPreference = (ListPreference) preference;
                int index = listPreference.findIndexOfValue(stringValue);

                // Set the summary to reflect the new value.
                preference.setSummary(
                        index >= 0
                                ? listPreference.getEntries()[index]
                                : null);
            } else {
                // For all other preferences, set the summary to the value's
                // simple string representation.
                preference.setSummary(stringValue);
            }
            return true;
        }
    };

    /**
     * Helper method to determine if the device has an extra-large screen. For
     * example, 10" tablets are extra-large.
     */
    private static boolean isXLargeTablet(Context context) {
        return (context.getResources().getConfiguration().screenLayout
                & Configuration.SCREENLAYOUT_SIZE_MASK) >= Configuration.SCREENLAYOUT_SIZE_XLARGE;
    }

    /**
     * Binds a preference's summary to its value. More specifically, when the
     * preference's value is changed, its summary (line of text below the
     * preference title) is updated to reflect the value. The summary is also
     * immediately updated upon calling this method. The exact display format is
     * dependent on the type of preference.
     *
     * @see #sBindPreferenceSummaryToValueListener
     */
    private static void bindPreferenceSummaryToValue(Preference preference) {
        // Set the listener to watch for value changes.
        preference.setOnPreferenceChangeListener(sBindPreferenceSummaryToValueListener);

        // Trigger the listener immediately with the preference's
        // current value.
        sBindPreferenceSummaryToValueListener.onPreferenceChange(preference,
                PreferenceManager
                        .getDefaultSharedPreferences(preference.getContext())
                        .getString(preference.getKey(), ""));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setupActionBar();

        // Display the fragment as the main content (to not use headers)
        getFragmentManager().beginTransaction().replace(android.R.id.content,
                new SettingsFragment()).commit();
    }

    /**
     * Set up the {@link android.app.ActionBar}, if the API is available.
     */
    private void setupActionBar() {
        ActionBar actionBar = getActionBar();
        if (actionBar != null) {
            // Show the Up button in the action bar.
            actionBar.setDisplayHomeAsUpEnabled(true);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean onIsMultiPane() {
        return isXLargeTablet(this);
    }

    /**
     * This method stops fragment injection in malicious applications.
     * Make sure to deny any unknown fragments here.
     */
    protected boolean isValidFragment(String fragmentName) {
        return PreferenceFragment.class.getName().equals(fragmentName)
                || SettingsFragment.class.getName().equals(fragmentName);
    }

    /**
     * This fragment shows general preferences only. It is used when the
     * activity is showing a two-pane settings UI.
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB)
    public static class SettingsFragment extends PreferenceFragment {
        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.pref_settings);
            setHasOptionsMenu(true);

            int nproc = Runtime.getRuntime().availableProcessors();
            findPreference("nthreads_value").setDefaultValue(Integer.toString(nproc));

            // Bind the summaries of EditText/List/Dialog/Ringtone preferences
            // to their values. When their values change, their summaries are
            // updated to reflect the new value, per the Android Design
            // guidelines.
            bindPreferenceSummaryToValue(findPreference("decimation_list"));
            bindPreferenceSummaryToValue(findPreference("sigma_value"));
            bindPreferenceSummaryToValue(findPreference("nthreads_value"));
            bindPreferenceSummaryToValue(findPreference("tag_family_list"));
            bindPreferenceSummaryToValue(findPreference("max_hamming_error"));
            bindPreferenceSummaryToValue(findPreference("apriltag_size"));
            bindPreferenceSummaryToValue(findPreference("calibration_fx"));
            bindPreferenceSummaryToValue(findPreference("calibration_fy"));
            bindPreferenceSummaryToValue(findPreference("calibration_cx"));
            bindPreferenceSummaryToValue(findPreference("calibration_cy"));

            // Populate camera preview resolutions dynamically
            ListPreference resolutionPref = (ListPreference) findPreference("preview_resolution");
            if (resolutionPref != null) {
                Camera camera = null;
                try {
                    int camidx = 0;
                    Camera.CameraInfo info = new Camera.CameraInfo();
                    for (int i = 0; i < Camera.getNumberOfCameras(); i += 1) {
                        Camera.getCameraInfo(i, info);
                        if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) {
                            camidx = i;
                            break;
                        }
                    }
                    camera = Camera.open(camidx);
                    Camera.Parameters parameters = camera.getParameters();
                    List<Camera.Size> supportedSizes = parameters.getSupportedPreviewSizes();
                    
                    if (supportedSizes != null && supportedSizes.size() > 0) {
                        ArrayList<String> entries = new ArrayList<>();
                        ArrayList<String> entryValues = new ArrayList<>();
                        
                        entries.add("Default (Largest)");
                        entryValues.add("largest");
                        
                        for (Camera.Size size : supportedSizes) {
                            if (size.width == size.height) continue;
                            String label = size.width + "x" + size.height;
                            if (size.width == 1920 && size.height == 1080) {
                                label += " (1080p)";
                            } else if (size.width == 1280 && size.height == 720) {
                                label += " (720p)";
                            } else if (size.width == 960 && size.height == 540) {
                                label += " (qHD)";
                            } else if (size.width == 640 && size.height == 480) {
                                label += " (VGA)";
                            }
                            entries.add(label);
                            entryValues.add(size.width + "x" + size.height);
                        }
                        
                        resolutionPref.setEntries(entries.toArray(new CharSequence[0]));
                        resolutionPref.setEntryValues(entryValues.toArray(new CharSequence[0]));
                        
                        if (resolutionPref.getValue() == null) {
                            resolutionPref.setValue("largest");
                        }
                    }
                } catch (Exception e) {
                    Log.e("SettingsActivity", "Error loading supported preview sizes: " + e.getMessage());
                    resolutionPref.setEntries(new CharSequence[]{"Default (Largest)"});
                    resolutionPref.setEntryValues(new CharSequence[]{"largest"});
                    resolutionPref.setValue("largest");
                } finally {
                    if (camera != null) {
                        camera.release();
                    }
                }
                bindPreferenceSummaryToValue(resolutionPref);
            }

            // Dynamic FRC Size locking behavior
            final Preference sizePref = findPreference("apriltag_size");
            final SwitchPreference frcPref = (SwitchPreference) findPreference("frc_mode");
            if (frcPref != null && sizePref != null) {
                sizePref.setEnabled(!frcPref.isChecked());
                frcPref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                    @Override
                    public boolean onPreferenceChange(Preference preference, Object newValue) {
                        boolean enabled = (Boolean) newValue;
                        sizePref.setEnabled(!enabled);
                        return true;
                    }
                });
            }
        }

        @Override
        public boolean onOptionsItemSelected(MenuItem item) {
            int id = item.getItemId();
            if (id == android.R.id.home) {
                getActivity().finish();
                return true;
            }
            return super.onOptionsItemSelected(item);
        }
    }
}
