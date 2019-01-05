package org.ghrobotics.lib.mathematics.statespace

class StateSpaceObserver(val coefficients: StateSpaceObserverCoefficients, val plant: StateSpacePlant) {

    var xHat = Matrix(plant.x.rowDimension, plant.x.columnDimension)

    fun predict(newU: Matrix) {
        xHat = plant.updateX(xHat, newU)
    }

    fun correct(u: Matrix, y: Matrix) {
        xHat += plant.coefficients.Ainv * coefficients.L * (y - plant.coefficients.C * xHat - plant.coefficients.D * u)
    }
}

data class StateSpaceObserverCoefficients(
    val L: Matrix
)