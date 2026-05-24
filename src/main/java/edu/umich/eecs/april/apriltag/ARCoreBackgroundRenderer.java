package edu.umich.eecs.april.apriltag;

import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import com.google.ar.core.Frame;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

/**
 * Renders the camera feed from an OpenGL external texture onto the GLSurfaceView.
 */
public class ARCoreBackgroundRenderer {
    private static final String VERTEX_SHADER_CODE =
        "attribute vec4 a_Position;\n" +
        "attribute vec2 a_TexCoord;\n" +
        "varying vec2 v_TexCoord;\n" +
        "void main() {\n" +
        "   gl_Position = a_Position;\n" +
        "   v_TexCoord = a_TexCoord;\n" +
        "}\n";

    private static final String FRAGMENT_SHADER_CODE =
        "#extension GL_OES_EGL_image_external : require\n" +
        "precision mediump float;\n" +
        "varying vec2 v_TexCoord;\n" +
        "uniform samplerExternalOES s_Texture;\n" +
        "void main() {\n" +
        "   gl_FragColor = texture2D(s_Texture, v_TexCoord);\n" +
        "}\n";

    private static final float[] QUAD_COORDS = {
        -1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         1.0f,  1.0f, 0.0f,
    };

    private static final float[] TEX_COORDS = {
        0.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 1.0f,
        1.0f, 0.0f,
    };

    private FloatBuffer mQuadCoords;
    private final FloatBuffer mQuadTexCoords = ByteBuffer.allocateDirect(TEX_COORDS.length * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer();
    private final FloatBuffer mTransformedTexCoords = ByteBuffer.allocateDirect(TEX_COORDS.length * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer();
    private int mProgram;
    private int mPositionHandle;
    private int mTexCoordHandle;
    private int mTextureId = -1;

    public void init() {
        // Texture generation
        int[] textures = new int[1];
        GLES20.glGenTextures(1, textures, 0);
        mTextureId = textures[0];
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, mTextureId);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);

        // Buffer setup
        ByteBuffer bb = ByteBuffer.allocateDirect(QUAD_COORDS.length * 4);
        bb.order(ByteOrder.nativeOrder());
        mQuadCoords = bb.asFloatBuffer();
        mQuadCoords.put(QUAD_COORDS);
        mQuadCoords.position(0);

        mQuadTexCoords.put(TEX_COORDS);
        mQuadTexCoords.position(0);

        int vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER_CODE);
        int fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_CODE);

        mProgram = GLES20.glCreateProgram();
        GLES20.glAttachShader(mProgram, vertexShader);
        GLES20.glAttachShader(mProgram, fragmentShader);
        GLES20.glLinkProgram(mProgram);
    }

    public int getTextureId() {
        return mTextureId;
    }

    public void draw(Frame frame) {
        mTransformedTexCoords.position(0);
        frame.transformDisplayUvCoords(mQuadTexCoords, mTransformedTexCoords);

        GLES20.glUseProgram(mProgram);

        mPositionHandle = GLES20.glGetAttribLocation(mProgram, "a_Position");
        mTexCoordHandle = GLES20.glGetAttribLocation(mProgram, "a_TexCoord");

        GLES20.glEnableVertexAttribArray(mPositionHandle);
        GLES20.glVertexAttribPointer(mPositionHandle, 3, GLES20.GL_FLOAT, false, 12, mQuadCoords);

        GLES20.glEnableVertexAttribArray(mTexCoordHandle);
        GLES20.glVertexAttribPointer(mTexCoordHandle, 2, GLES20.GL_FLOAT, false, 8, mTransformedTexCoords);

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, mTextureId);
        GLES20.glUniform1i(GLES20.glGetUniformLocation(mProgram, "s_Texture"), 0);

        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);

        GLES20.glDisableVertexAttribArray(mPositionHandle);
        GLES20.glDisableVertexAttribArray(mTexCoordHandle);
    }

    private static int loadShader(int type, String shaderCode) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, shaderCode);
        GLES20.glCompileShader(shader);
        return shader;
    }
}
