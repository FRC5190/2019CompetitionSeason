package org.ghrobotics.lib.mathematics.statespace

/**
 * Implements a state space controller with the control law:
 * u = K(r - x) * Kff(nextR - Ar)
 */
class StateSpaceController(val coefficients: StateSpaceControllerCoefficients, val plant: StateSpacePlant) {

    var u: Matrix = Matrix(plant.coefficients.B.columnDimension, 1)
    var r: Matrix = Matrix(plant.coefficients.A.rowDimension, 1)

    fun update(x: Matrix) {
        u = coefficients.K * (r - x) + coefficients.Kff * (r - plant.coefficients.A * r)
    }

    fun update(nextR: Matrix, x: Matrix) {
        u = coefficients.K * (r - x) + coefficients.Kff * (nextR - plant.coefficients.A * r)
        r = nextR
    }

}

/**
 * Co-efficients for the state space controller.
 *
 * @param K Gain matrix generated from discrete LQR.
 * @param Kff Feedforward matrix generated from the Moore-Pemrose pseudoinverse of the B matrix.
 * @param uMin Minimum control input
 * @param uMax Maximum control input
 */
data class StateSpaceControllerCoefficients(
    val K: Matrix,
    val Kff: Matrix
)