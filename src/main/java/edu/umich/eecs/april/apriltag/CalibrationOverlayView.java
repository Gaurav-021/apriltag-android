package edu.umich.eecs.april.apriltag;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class CalibrationOverlayView extends View {
    private float[] mCorners = null;
    private int mPatternRows = 0;
    private int mPatternCols = 0;
    
    private final Paint mPointPaint = new Paint();
    private final Paint mLinePaint = new Paint();
    
    public CalibrationOverlayView(Context context) {
        super(context);
        init();
    }
    
    public CalibrationOverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }
    
    private void init() {
        mPointPaint.setColor(Color.RED);
        mPointPaint.setStyle(Paint.Style.FILL);
        mPointPaint.setAntiAlias(true);
        
        mLinePaint.setColor(0xFF39FF14); // Neon green
        mLinePaint.setStyle(Paint.Style.STROKE);
        mLinePaint.setStrokeWidth(3.0f);
        mLinePaint.setAntiAlias(true);
    }
    
    public synchronized void setCorners(float[] corners, int rows, int cols) {
        mCorners = corners;
        mPatternRows = rows;
        mPatternCols = cols;
        postInvalidate();
    }
    
    public synchronized void clearCorners() {
        mCorners = null;
        postInvalidate();
    }
    
    @Override
    protected synchronized void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (mCorners == null || mCorners.length == 0) return;
        
        int numPoints = mCorners.length / 2;
        
        // Draw connecting lines row by row
        for (int r = 0; r < mPatternRows; r++) {
            for (int c = 0; c < mPatternCols - 1; c++) {
                int idx1 = (r * mPatternCols + c) * 2;
                int idx2 = (r * mPatternCols + c + 1) * 2;
                if (idx2 < mCorners.length) {
                    canvas.drawLine(mCorners[idx1], mCorners[idx1 + 1],
                                    mCorners[idx2], mCorners[idx2 + 1], mLinePaint);
                }
            }
        }
        
        // Draw connecting lines column by column
        for (int c = 0; c < mPatternCols; c++) {
            for (int r = 0; r < mPatternRows - 1; r++) {
                int idx1 = (r * mPatternCols + c) * 2;
                int idx2 = ((r + 1) * mPatternCols + c) * 2;
                if (idx2 < mCorners.length) {
                    canvas.drawLine(mCorners[idx1], mCorners[idx1 + 1],
                                    mCorners[idx2], mCorners[idx2 + 1], mLinePaint);
                }
            }
        }
        
        // Draw points with gradient colors (from Red to Cyan)
        for (int i = 0; i < numPoints; i++) {
            float ratio = (float) i / numPoints;
            int red = (int) ((1.0f - ratio) * 255);
            int green = (int) (ratio * 255);
            int blue = (int) (ratio * 255);
            mPointPaint.setColor(Color.rgb(red, green, blue));
            
            canvas.drawCircle(mCorners[i * 2], mCorners[i * 2 + 1], 8.0f, mPointPaint);
        }
    }
}
