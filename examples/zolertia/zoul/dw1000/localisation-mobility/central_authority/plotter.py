
from packets import IPv6Address
from multilateration import Coordinates
import csv
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt


class Plotter(ABC):
    
    def __init__(self, datafile: str, header: list, _write: bool = False):
        self.datafile = datafile
        self._write = _write

        self.NUM_BINS = 500

        if _write:
            # open the datafile file as a CSV file
            self.f = open(self.datafile, 'w')
            self.writer = csv.writer(self.f)
            self.writer.writerow(header)
            self.f.flush()

    def write(self, line: list):
        assert self._write
        # save the new row in the csv file
        if self.f.tell() < 2**31:
            # stop writing when the file exceeds 2GB
            self.writer.writerow(line)
            self.f.flush()
    
    def close(self):
        assert self._write
        self.f.close()

    @abstractmethod
    def plot(self, anchor_to_plot: IPv6Address, expected_mean = None):
        pass

class PropagationTimePlotter(Plotter):

    def __init__(self, datafile: str, _write: bool = True):
        super().__init__(datafile, ['anchor', 'propagation_time'], _write)

    def write(self, anchor: IPv6Address, propagation_time: float):
        return super().write([anchor, propagation_time])

    
    def plot(self, anchor_to_plot: IPv6Address, expected_mean: int = None):
        # plot a hist graph
        x = []

        with open(self.datafile) as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['anchor'] == str(anchor_to_plot):
                    x.append(int(row['propagation_time']))

        # the histogram of the data
        n, bins, patches = plt.hist(x, self.NUM_BINS, density=True, facecolor='g', alpha=0.75)

        plt.xlabel('Propagation Time')
        # plt.ylabel('Probability')
        plt.title(f'Propagation Times of {anchor_to_plot}')
        plt.grid(True)
        if expected_mean is not None:
            plt.axvline(expected_mean, color='k', linestyle='dashed', linewidth=1)
        plt.show()

        



class GeolocationPlotter(Plotter):

    def __init__(self, datafile: str, _write: bool = True):
        super().__init__(datafile, ['anchor', 'x', 'y'], _write)

    def write(self, anchor: IPv6Address, coordinates: Coordinates):
        return super().write([anchor, coordinates.x, coordinates.y])
    
    
    def plot(self, anchor_to_plot: IPv6Address, expected_mean: Coordinates = None):
        # plot two histograms side by side
        x = []
        y = []

        with open(self.datafile) as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['anchor'] == str(anchor_to_plot):
                    x.append(float(row['x']))
                    y.append(float(row['y']))
        
        # side by side histograms
        fig, axs = plt.subplots(1, 2)
        axs[0].hist(x, self.NUM_BINS, density=True, facecolor='g', alpha=0.75)
        axs[0].set_title(f'X Coordinates of {anchor_to_plot}')
        axs[0].set_xlabel('x')
        if expected_mean is not None:
            axs[0].axvline(expected_mean.x, color='k', linestyle='dashed', linewidth=1)


        axs[1].hist(y, self.NUM_BINS, density=True, facecolor='r', alpha=0.75)
        axs[1].set_title(f'Y Coordinates of {anchor_to_plot}')
        axs[1].set_xlabel('y')
        if expected_mean is not None:
            axs[1].axvline(expected_mean.y, color='k', linestyle='dashed', linewidth=1)


        plt.show()


if __name__ == "__main__":
    a = IPv6Address("fe80000000000000fdffffffffff0001")
    
    
    plotter = PropagationTimePlotter("test.csv")
    plotter.write(a, 5.2)
    plotter.write(a, 5.3)
    plotter.write(a, 5.1)
    plotter.write(a, 5.3)
    plotter.write(a, 5.3)
    plotter.close()

    plotter = PropagationTimePlotter("test.csv", False)
    plotter.plot(a, expected_mean=5.2)

    plotter = GeolocationPlotter("test.csv")
    plotter.write(a, Coordinates(4.1, 5.1))
    plotter.write(a, Coordinates(4.2, 5.2))
    plotter.write(a, Coordinates(4.3, 5.3))
    plotter.write(a, Coordinates(4.4, 5.4))
    plotter.close()

    plotter = GeolocationPlotter("test.csv", False)
    plotter.plot(a, expected_mean=Coordinates(4.2, 5.2))
