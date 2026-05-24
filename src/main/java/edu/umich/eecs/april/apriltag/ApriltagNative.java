package edu.umich.eecs.april.apriltag;

import android.graphics.Bitmap;
import android.util.Log;

import java.util.ArrayList;

/**
 * Interface to native C AprilTag library.
 */

public class ApriltagNative {
    static {
        System.loadLibrary("apriltag");
        native_init();
    }

    public static native void native_init();

    public static native void yuv_to_rgb(byte[] src, int width, int height, Bitmap dst);

    public static native void apriltag_init(String tagFamily, int errorBits, double decimateFactor,
                                            double blurSigma, int nthreads);

    public static native ArrayList<ApriltagDetection> apriltag_detect_yuv(byte[] src, int width, int height);

    public static native double[] apriltag_detect_yuv_flat(byte[] src, int width, int height);

    public static ArrayList<ApriltagDetection> apriltag_detect_yuv_flat_decoded(byte[] src, int width, int height) {
        double[] flat = apriltag_detect_yuv_flat(src, width, height);
        ArrayList<ApriltagDetection> list = new ArrayList<>();
        if (flat == null || flat.length == 0) {
            return list;
        }
        int count = (int) flat[0];
        for (int i = 0; i < count; i++) {
            ApriltagDetection det = new ApriltagDetection();
            int offset = 1 + i * 12;
            det.id = (int) flat[offset + 0];
            det.hamming = (int) flat[offset + 1];
            det.c = new double[]{flat[offset + 2], flat[offset + 3]};
            det.p = new double[]{
                flat[offset + 4], flat[offset + 5],
                flat[offset + 6], flat[offset + 7],
                flat[offset + 8], flat[offset + 9],
                flat[offset + 10], flat[offset + 11]
            };
            list.add(det);
        }
        return list;
    }
}
