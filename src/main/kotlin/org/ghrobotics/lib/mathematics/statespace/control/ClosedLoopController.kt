package org.ghrobotics.lib.mathematics.statespace.control

import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.ghrobotics.lib.mathematics.statespace.observers.Observer

/**
 * Implementation of the most basic state space controller.
 * The [_closedLoopOutput] function is expected to be called at the same dt with which the gain matrix
 * was generated using LQR.
 */
@Suppress("ConstructorParameterNaming")
abstract class ClosedLoopController(
    protected val K: Matrix,
    private val observer: Observer
) {
    private var u = Matrix(K.rowDimension, 1)

    @Suppress("FunctionNaming", "FunctionName")
    protected abstract fun _closedLoopOutput(r: Matrix, xHat: Matrix): Matrix

    fun getClosedLoopOutput(r: Matrix, y: Matrix): Matrix {
        u = _closedLoopOutput(r, observer.correct(u, y))
        observer.predict(u)

        return u
    }
}

typealias Matrix = Array2DRowRealMatrix

operator fun Matrix.plus(other: Matrix): Array2DRowRealMatrix = this.add(other)

operator fun Matrix.minus(other: Matrix): Array2DRowRealMatrix = this.subtract(other)

operator fun Matrix.times(other: Matrix): Array2DRowRealMatrix = this.multiply(other)