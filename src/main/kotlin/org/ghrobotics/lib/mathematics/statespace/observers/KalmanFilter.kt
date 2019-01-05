package org.ghrobotics.lib.mathematics.statespace.observers

import org.apache.commons.math3.linear.MatrixUtils
import org.ghrobotics.lib.mathematics.statespace.control.Matrix
import org.ghrobotics.lib.mathematics.statespace.control.minus
import org.ghrobotics.lib.mathematics.statespace.control.plus
import org.ghrobotics.lib.mathematics.statespace.control.times

@Suppress("ConstructorParameterNaming")
/**
 * Represents a Kalman Filter
 */
class KalmanFilter(
    private val L: Matrix,
    private val A: Matrix,
    private val B: Matrix,
    private val C: Matrix,
    private val D: Matrix,
    initialSystemState: Matrix
) : Observer {

    private var xHat = initialSystemState
    val aInv = Matrix(MatrixUtils.inverse(A).data)

    override fun predict(u: Matrix): Matrix {
        xHat = A * xHat + B * u
        return xHat
    }

    override fun correct(u: Matrix, y: Matrix): Matrix {
        xHat += aInv * L * (y - C * xHat - D * u)
        return xHat
    }
}