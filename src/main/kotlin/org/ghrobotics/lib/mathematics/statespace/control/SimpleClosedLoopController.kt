package org.ghrobotics.lib.mathematics.statespace.control

import org.ghrobotics.lib.mathematics.statespace.observers.Observer

/**
 * The [getClosedLoopOutput] function is expected to be called at the same dt with which the gain matrix
 * was generated using LQR.
 */
@Suppress("ConstructorParameterNaming")
class SimpleClosedLoopController(
    K: Matrix,
    observer: Observer
) : ClosedLoopController(K, observer) {
    override fun _closedLoopOutput(r: Matrix, xHat: Matrix): Matrix = K * (r - xHat)
}