package org.ghrobotics.lib.mathematics.statespace.control

import org.ghrobotics.lib.mathematics.statespace.observers.Observer

/**
 * The [getClosedLoopOutput] function is expected to be called at the same dt with which the gain matrix
 * was generated using LQR.
 */
@Suppress("ConstructorParameterNaming")
class TwoStateFFClosedLoopController(
    K: Matrix,
    observer: Observer,
    private val A: Matrix,
    private val KFF: Matrix
) : ClosedLoopController(K, observer) {

    override fun _closedLoopOutput(r: Matrix, xHat: Matrix): Matrix = K * (r - xHat) + outputFromFeedForward(r)

    private var currentR = Matrix(A.rowDimension, 1)

    private fun outputFromFeedForward(nextR: Matrix): Matrix =
        KFF.multiply(nextR - A * currentR).also { currentR = nextR }
}
