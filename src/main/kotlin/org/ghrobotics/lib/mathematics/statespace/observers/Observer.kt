package org.ghrobotics.lib.mathematics.statespace.observers

import org.ghrobotics.lib.mathematics.statespace.control.Matrix

interface Observer {
    fun predict(u: Matrix): Matrix
    fun correct(u: Matrix, y: Matrix): Matrix
}