
"""
Source: https://github.com/glucee/Multilateration/


This script implements a multilateration algorithm that, given the coordinates of a finite number of radio stations,
and given their distances to the station (derived from the intensities of the signal they received in a previous step)
computes the most probable coordinates of the station. Even if the distances computed for each station do not match
(in terms of pointing to a single optimal solution) the algorithm finds the coordinates that minimize the error function
and returns the most optimal solution possible.


https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
https://docs.scipy.org/doc/scipy/reference/optimize.minimize-neldermead.html#optimize-minimize-neldermead

"""

from dataclasses import dataclass
import time
from scipy.optimize import minimize
import numpy as np
from packets import Anchor

@dataclass
class TwoWayRangingResult:

	station: Anchor
	"""
	Anchor with which the two-way ranging was performed.
	"""

	received_time: float
	"""
	time when the result was received, in seconds since the Epoch.
	"""
	
	distance: float
	"""
	Distance between the specified anchor and the mobile, in metres.
	"""

@dataclass
class Coordinates:
	x: float
	y: float

	@classmethod
	def from_list(cls, lst: list):
		assert len(lst) >= 2
		return Coordinates(lst[0], lst[1])
	
	def __str__(self) -> str:
		return f"[x={self.x}, y={self.y}]"


class MultilaterationAlgorithm:

	def __init__(self):
		self.anchors = Anchor.all_anchors()
		self.previous_results: dict[Anchor, TwoWayRangingResult] = {}		# priority Queue ? Keep the most fresh
	

	def gps_solve_on_result(self, distance: float, station: Anchor) -> Coordinates:
		result = TwoWayRangingResult(station, time.time(), distance)

		self.previous_results[station] = result
		
		if len(self.previous_results) < 3:
			return None		# Not enough data for Multilateration

		

		three_last_anchors = sorted(self.previous_results, key=lambda anchor: self.previous_results[anchor].received_time, reverse=True)[:3]
		three_last_distances = [self.previous_results[a].distance for a in three_last_anchors]
		three_last_coordinates = list(map(lambda anchor: np.array([anchor.x_pos, anchor.y_pos]), three_last_anchors))

		return Coordinates.from_list(self.gps_solve(three_last_distances, three_last_coordinates))

		


	@classmethod
	def gps_solve(cls, distances_to_station, stations_coordinates):
		def error(x, c, r):
			return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

		l = len(stations_coordinates)
		S = sum(distances_to_station)
		# compute weight vector for initial guess
		W = [((l - 1) * S) / (S - w) for w in distances_to_station]
		# get initial guess of point location
		x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
		# optimize distance from signal origin to border of spheres
		return minimize(error, x0, args=(stations_coordinates, distances_to_station), method='Nelder-Mead').x


if __name__ == "__main__":
	stations = list(np.array([[1,1], [0,1], [1,0], [0,0]]))
	distances_to_station = [0.1, 0.5, 0.5, 1.3]
	print(MultilaterationAlgorithm.gps_solve(distances_to_station, stations))
