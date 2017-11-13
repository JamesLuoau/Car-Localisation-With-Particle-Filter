import unittest
from update_measurement import UpdateMeasurement
from numpy import testing


class LocalisationTest(unittest.TestCase):

    def test_sense(self):
        update_measurement = UpdateMeasurement()
        q = update_measurement.sense(prior_prob_distribution_p=[0.2, 0.2, 0.2, 0.2, 0.2], measurement_Z="red")
        testing.assert_almost_equal(q, [0.111, 0.333, 0.333, 0.111, 0.111], decimal=3)

    def test_move_exact(self):
        q = UpdateMeasurement.move_exact(input_distribution_p=[0, 1, 0, 0], number_of_steps_U=1)
        self.assertEqual(q, [0, 0, 1, 0])

    def test_move_exact(self):
        update_measurement = UpdateMeasurement()
        q = update_measurement.move_inexact(input_distribution_p=[0, 1, 0, 0, 0], number_of_steps_U=1)
        self.assertEqual(q, [0, 0.1, 0.8, 0.1, 0])

        q = update_measurement.move_inexact(input_distribution_p=q, number_of_steps_U=1)
        testing.assert_almost_equal(q, [0.01, 0.01, 0.16, 0.66, 0.16], decimal=3)

        for k in range(1000):
            q = update_measurement.move_inexact(input_distribution_p=q, number_of_steps_U=1)
        testing.assert_almost_equal(q, [0.2, 0.2, 0.2, 0.2, 0.2], decimal=3)

    def test_sense_and_move(self):
        update_measurement = UpdateMeasurement(world=['green', 'red', 'red', 'green', 'green'])
        q = update_measurement.sense_and_move(motions=[1, 1], measurements=["red", "green"])
        testing.assert_almost_equal(q, [0.211, 0.152, 0.081, 0.168, 0.387], decimal=3)

        q = update_measurement.sense_and_move(motions=[1, 1], measurements=["red", "red"])
        testing.assert_almost_equal(q, [0.079, 0.075, 0.225, 0.433, 0.188], decimal=3)
