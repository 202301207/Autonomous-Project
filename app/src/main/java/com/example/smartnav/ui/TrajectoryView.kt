package com.example.smartnav.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import com.example.smartnav.data.Pose2D
import kotlin.math.max
import kotlin.math.min

/**
 * Renders top-down 2D trajectories for side-by-side comparison of DR vs SLAM.
 */
class TrajectoryView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val drPoints = mutableListOf<Pose2D>()
    private val slamPoints = mutableListOf<Pose2D>()

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.DKGRAY
        strokeWidth = 1f
        style = Paint.Style.STROKE
    }

    private val drPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#2962FF")
        strokeWidth = 6f
        style = Paint.Style.STROKE
    }

    private val slamPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#D50000")
        strokeWidth = 6f
        style = Paint.Style.STROKE
    }

    fun setTrajectories(dr: List<Pose2D>, slam: List<Pose2D>) {
        drPoints.apply {
            clear()
            addAll(dr)
        }
        slamPoints.apply {
            clear()
            addAll(slam)
        }
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (drPoints.isEmpty() && slamPoints.isEmpty()) {
            drawCenterText(canvas, "Paths will appear here")
            return
        }
        drawGrid(canvas)
        drawPath(canvas, drPoints, drPaint)
        drawPath(canvas, slamPoints, slamPaint)
    }

    private fun drawGrid(canvas: Canvas) {
        val step = width.toFloat() / 10f
        for (i in 0..10) {
            val x = i * step
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaint)
        }
        val yStep = height.toFloat() / 10f
        for (i in 0..10) {
            val y = i * yStep
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaint)
        }
    }

    private fun drawPath(canvas: Canvas, points: List<Pose2D>, paint: Paint) {
        if (points.size < 2) return
        val bounds = computeBounds()
        val scale = computeScale(bounds)
        val offsetX = width / 2f
        val offsetY = height / 2f
        val path = Path()
        points.forEachIndexed { index, pose ->
            val x = (pose.x - bounds.centerX) * scale + offsetX
            val y = (pose.y - bounds.centerY) * scale + offsetY
            if (index == 0) path.moveTo(x, y) else path.lineTo(x, y)
        }
        canvas.drawPath(path, paint)
    }

    private fun computeScale(bounds: Bounds): Float {
        val maxSpan = max(bounds.width, bounds.height).coerceAtLeast(1f)
        val drawableSize = min(width, height) * 0.8f
        return drawableSize / maxSpan
    }

    private fun computeBounds(): Bounds {
        var minX = 0f
        var maxX = 0f
        var minY = 0f
        var maxY = 0f
        (drPoints + slamPoints).forEach {
            minX = min(minX, it.x)
            maxX = max(maxX, it.x)
            minY = min(minY, it.y)
            maxY = max(maxY, it.y)
        }
        val width = maxX - minX
        val height = maxY - minY
        return Bounds(width, height, (maxX + minX) / 2f, (maxY + minY) / 2f)
    }

    private fun drawCenterText(canvas: Canvas, text: String) {
        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 36f
            textAlign = Paint.Align.CENTER
        }
        canvas.drawText(text, width / 2f, height / 2f, textPaint)
    }

    private data class Bounds(
        val width: Float,
        val height: Float,
        val centerX: Float,
        val centerY: Float
    )
}

