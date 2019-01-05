package org.ghrobotics.lib.mathematics.statespace

class StateSpacePlant(val coefficients: StateSpacePlantCoefficients) {

    var x: Matrix = Matrix(coefficients.A.rowDimension, 1)
    var y: Matrix = Matrix(coefficients.C.rowDimension, 1)

    fun update(u: Matrix) {
        x = updateX(x, u)
        y = updateY(u)
    }

    fun updateX(x: Matrix, u: Matrix) = coefficients.A * x + coefficients.B * u
    fun updateY(u: Matrix) = coefficients.C * x + coefficients.D * u

}

/**
 * Co-efficients for a state space plant.
 * @param A System matrix
 * @param Ainv Inverse of system matrix
 * @param B Input matrix
 * @param C Output matrix
 * @param D Feedthrough matrix
 */
data class StateSpacePlantCoefficients(
    val A: Matrix,
    val Ainv: Matrix,
    val B: Matrix,
    val C: Matrix,
    val D: Matrix
)