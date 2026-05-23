package edu.umich.eecs.april.apriltag;

public class FRCTagLayout {
    // 1-indexed array of 4x4 matrices for the 32 FRC AprilTags
    public static final double[][][] TAG_TRANSFORMS = new double[33][4][4];

    static {
        // Tag 1
        TAG_TRANSFORMS[1] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 3.604958999999999},
            {1.2246467991473532e-16, -1, 0, 3.3899913999999995},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 2
        TAG_TRANSFORMS[2] = new double[][]{
            {-2.220446049250313e-16, -1.0000000000000002, 0, 3.6423986},
            {1.0000000000000002, -2.220446049250313e-16, 0, 0.6032558000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 3
        TAG_TRANSFORMS[3] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 3.0388438000000004},
            {1.2246467991473532e-16, -1, 0, 0.35545340000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 4
        TAG_TRANSFORMS[4] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 3.0388438000000004},
            {1.2246467991473532e-16, -1, 0, -0.0001465999999998857},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 5
        TAG_TRANSFORMS[5] = new double[][]{
            {-2.220446049250313e-16, 1, 0, 3.6423986},
            {-1, -2.220446049250313e-16, 0, -0.6035489999999997},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 6
        TAG_TRANSFORMS[6] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 3.604958999999999},
            {1.2246467991473532e-16, -1, 0, -3.3902845999999998},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 7
        TAG_TRANSFORMS[7] = new double[][]{
            {1, 0, 0, 3.679863599999999},
            {0, 1, 0, -3.3902845999999998},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 8
        TAG_TRANSFORMS[8] = new double[][]{
            {-2.220446049250313e-16, 1, 0, 3.997998599999999},
            {-1, -2.220446049250313e-16, 0, -0.6035489999999997},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 9
        TAG_TRANSFORMS[9] = new double[][]{
            {1, 0, 0, 4.246156599999999},
            {0, 1, 0, -0.3557465999999998},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 10
        TAG_TRANSFORMS[10] = new double[][]{
            {1, 0, 0, 4.246156599999999},
            {0, 1, 0, -0.0001465999999998857},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 11
        TAG_TRANSFORMS[11] = new double[][]{
            {-2.220446049250313e-16, -1.0000000000000002, 0, 3.997998599999999},
            {1.0000000000000002, -2.220446049250313e-16, 0, 0.6032558000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 12
        TAG_TRANSFORMS[12] = new double[][]{
            {1, 0, 0, 3.679863599999999},
            {0, 1, 0, 3.3899913999999995},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 13
        TAG_TRANSFORMS[13] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 8.240331999999999},
            {1.2246467991473532e-16, -1, 0, 3.3704079999999994},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 14
        TAG_TRANSFORMS[14] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 8.240331999999999},
            {1.2246467991473532e-16, -1, 0, 2.9386079999999994},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 15
        TAG_TRANSFORMS[15] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 8.2399764},
            {1.2246467991473532e-16, -1, 0, 0.29098820000000014},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 16
        TAG_TRANSFORMS[16] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, 8.2399764},
            {1.2246467991473532e-16, -1, 0, -0.14081180000000026},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 17
        TAG_TRANSFORMS[17] = new double[][]{
            {1, 0, 0, -3.6099364000000005},
            {0, 1, 0, -3.3902845999999998},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 18
        TAG_TRANSFORMS[18] = new double[][]{
            {-2.220446049250313e-16, 1, 0, -3.6474014000000006},
            {-1, -2.220446049250313e-16, 0, -0.6035489999999997},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 19
        TAG_TRANSFORMS[19] = new double[][]{
            {1, 0, 0, -3.0438466},
            {0, 1, 0, -0.3557465999999998},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 20
        TAG_TRANSFORMS[20] = new double[][]{
            {1, 0, 0, -3.0438466},
            {0, 1, 0, -0.0001465999999998857},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 21
        TAG_TRANSFORMS[21] = new double[][]{
            {-2.220446049250313e-16, -1.0000000000000002, 0, -3.6474014000000006},
            {1.0000000000000002, -2.220446049250313e-16, 0, 0.6032558000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 22
        TAG_TRANSFORMS[22] = new double[][]{
            {1, 0, 0, -3.6099364000000005},
            {0, 1, 0, 3.3899913999999995},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 23
        TAG_TRANSFORMS[23] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, -3.6848410000000005},
            {1.2246467991473532e-16, -1, 0, 3.3899913999999995},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 24
        TAG_TRANSFORMS[24] = new double[][]{
            {-2.220446049250313e-16, -1.0000000000000002, 0, -4.0030014000000005},
            {1.0000000000000002, -2.220446049250313e-16, 0, 0.6032558000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 25
        TAG_TRANSFORMS[25] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, -4.251134},
            {1.2246467991473532e-16, -1, 0, 0.35545340000000003},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 26
        TAG_TRANSFORMS[26] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, -4.251134},
            {1.2246467991473532e-16, -1, 0, -0.0001465999999998857},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 27
        TAG_TRANSFORMS[27] = new double[][]{
            {-2.220446049250313e-16, 1, 0, -4.0030014000000005},
            {-1, -2.220446049250313e-16, 0, -0.6035489999999997},
            {0, 0, 1, 1.12395},
            {0, 0, 0, 1}
        };
        // Tag 28
        TAG_TRANSFORMS[28] = new double[][]{
            {-1, -1.2246467991473532e-16, 0, -3.6848410000000005},
            {1.2246467991473532e-16, -1, 0, -3.3902845999999998},
            {0, 0, 1, 0.889},
            {0, 0, 0, 1}
        };
        // Tag 29
        TAG_TRANSFORMS[29] = new double[][]{
            {1, 0, 0, -8.2453094},
            {0, 1, 0, -3.3707266},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 30
        TAG_TRANSFORMS[30] = new double[][]{
            {1, 0, 0, -8.2453094},
            {0, 1, 0, -2.9389265999999994},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 31
        TAG_TRANSFORMS[31] = new double[][]{
            {1, 0, 0, -8.244953800000001},
            {0, 1, 0, -0.29130679999999964},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
        // Tag 32
        TAG_TRANSFORMS[32] = new double[][]{
            {1, 0, 0, -8.244953800000001},
            {0, 1, 0, 0.14049319999999987},
            {0, 0, 1, 0.55245},
            {0, 0, 0, 1}
        };
    }

    public static class CameraPose {
        public double x;
        public double y;
        public double z;
        public double roll;
        public double pitch;
        public double yaw;

        public CameraPose(double x, double y, double z, double roll, double pitch, double yaw) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }
    }

    public static CameraPose computeCameraPoseOnField(int tagId, double[] r1, double[] r2, double[] r3, double[] t) {
        if (tagId < 1 || tagId > 32) return null;
        double[][] T_field_tag = TAG_TRANSFORMS[tagId];
        if (T_field_tag == null) return null;

        // T_camera_tag = [ R_c_t | t ]
        // R_c_t columns are r1, r2, r3, so R_c_t row i is [r1[i], r2[i], r3[i]]
        // R_t_c = R_c_t^T (transpose), which means row i of R_t_c is r_i (r1, r2, r3 respectively)
        
        // t_t_c = -R_t_c * t
        double tx_t_c = -(r1[0]*t[0] + r1[1]*t[1] + r1[2]*t[2]);
        double ty_t_c = -(r2[0]*t[0] + r2[1]*t[1] + r2[2]*t[2]);
        double tz_t_c = -(r3[0]*t[0] + r3[1]*t[1] + r3[2]*t[2]);

        // T_field_camera = T_field_tag * T_camera_tag^-1
        // T_field_tag = [ R_f_t | t_f_t ]
        // t_f_c = R_f_t * t_t_c + t_f_t
        double x_field = T_field_tag[0][0]*tx_t_c + T_field_tag[0][1]*ty_t_c + T_field_tag[0][2]*tz_t_c + T_field_tag[0][3];
        double y_field = T_field_tag[1][0]*tx_t_c + T_field_tag[1][1]*ty_t_c + T_field_tag[1][2]*tz_t_c + T_field_tag[1][3];
        double z_field = T_field_tag[2][0]*tx_t_c + T_field_tag[2][1]*ty_t_c + T_field_tag[2][2]*tz_t_c + T_field_tag[2][3];

        // R_f_c = R_f_t * R_t_c
        // R_t_c row j is r_{j+1} (r1, r2, r3)
        // R_f_c[i][j] = T_field_tag[i][0]*r1[j] + T_field_tag[i][1]*r2[j] + T_field_tag[i][2]*r3[j]
        double r00_f_c = T_field_tag[0][0]*r1[0] + T_field_tag[0][1]*r2[0] + T_field_tag[0][2]*r3[0];
        double r01_f_c = T_field_tag[0][0]*r1[1] + T_field_tag[0][1]*r2[1] + T_field_tag[0][2]*r3[1];
        double r02_f_c = T_field_tag[0][0]*r1[2] + T_field_tag[0][1]*r2[2] + T_field_tag[0][2]*r3[2];

        double r10_f_c = T_field_tag[1][0]*r1[0] + T_field_tag[1][1]*r2[0] + T_field_tag[1][2]*r3[0];
        double r11_f_c = T_field_tag[1][0]*r1[1] + T_field_tag[1][1]*r2[1] + T_field_tag[1][2]*r3[1];
        double r12_f_c = T_field_tag[1][0]*r1[2] + T_field_tag[1][1]*r2[2] + T_field_tag[1][2]*r3[2];

        double r20_f_c = T_field_tag[2][0]*r1[0] + T_field_tag[2][1]*r2[0] + T_field_tag[2][2]*r3[0];
        double r21_f_c = T_field_tag[2][0]*r1[1] + T_field_tag[2][1]*r2[1] + T_field_tag[2][2]*r3[1];
        double r22_f_c = T_field_tag[2][0]*r1[2] + T_field_tag[2][1]*r2[2] + T_field_tag[2][2]*r3[2];

        // Extract Euler Angles (Roll, Pitch, Yaw) from R_f_c
        double pitch = Math.asin(-r02_f_c);
        double roll, yaw;
        if (Math.cos(pitch) > 1e-4) {
            roll = Math.atan2(r12_f_c, r22_f_c);
            yaw = Math.atan2(r01_f_c, r00_f_c);
        } else {
            roll = 0.0;
            yaw = Math.atan2(-r10_f_c, r11_f_c);
        }

        return new CameraPose(x_field, y_field, z_field, Math.toDegrees(roll), Math.toDegrees(pitch), Math.toDegrees(yaw));
    }
}
