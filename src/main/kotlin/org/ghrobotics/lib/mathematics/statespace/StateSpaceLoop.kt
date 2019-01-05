package org.ghrobotics.lib.mathematics.statespace

import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.MatrixUtils

class StateSpaceLoop(
    plantCoefficients: StateSpacePlantCoefficients,
    controllerCoefficients: StateSpaceControllerCoefficients,
    observerCoefficients: StateSpaceObserverCoefficients
) {
    private val plant = StateSpacePlant(plantCoefficients)
    private val controller = StateSpaceController(controllerCoefficients, plant)
    private val observer = StateSpaceObserver(observerCoefficients, plant)

    var nextR: Matrix = Matrix(plant.coefficients.A.rowDimension, 1)

    val error get() = controller.r - observer.xHat

    fun correct(y: Matrix) {
        observer.correct(controller.u, y)
    }

    fun update(): Matrix {
        controller.update(nextR, observer.xHat)
        observer.predict(controller.u)
        return controller.u
    }
}

typealias Matrix = Array2DRowRealMatrix

operator fun Matrix.plus(other: Matrix): Array2DRowRealMatrix = this.add(other)
operator fun Matrix.minus(other: Matrix): Array2DRowRealMatrix = this.subtract(other)
operator fun Matrix.times(other: Matrix): Array2DRowRealMatrix = this.multiply(other)
fun Matrix.inv() = Matrix(MatrixUtils.inverse(this).data)
