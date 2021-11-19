
from typing import Counter, List
from packets import IPv6Address
from multilateration import Coordinates
import csv
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator


class Plotter(ABC):
    
    def __init__(self, datafile: str, header: list, _write: bool = False):
        self.datafile = datafile
        self._write = _write

        self.MEASURE_COLOUR = '#1f77b4'
        self.ESTIMATE_COLOUR = '#ff7f0e'

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
    def plot(self, anchors_to_plot: List[IPv6Address], expected_mean = None):
        pass

class PropagationTimePlotter(Plotter):

    def __init__(self, datafile: str, _write: bool = True):
        super().__init__(datafile, ['anchor', 'propagation_time'], _write)

    def write(self, anchor: IPv6Address, propagation_time: float):
        return super().write(["".join(str(anchor).split(":")[-2:]), propagation_time])

    
    def plot(self, anchors_to_plot: List[IPv6Address], expected_means: int = None):
        # plot a hist graph
        i = 0
        if expected_means is None:
            expected_means = [None] * len(anchors_to_plot)

        fig, axs = plt.subplots(len(anchors_to_plot), 1)

        # add spacing between subplots
        fig.subplots_adjust(hspace=0.5)

        for anchor_to_plot, expected_mean in zip(anchors_to_plot, expected_means):
            x = []

            with open(self.datafile) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row['anchor'] == "".join(str(anchor_to_plot).split(":")[-2:]):
                        x.append(int(row['propagation_time']))
                
                # Remove outliers
                if "".join(str(anchor_to_plot).split(":")[-2:]) == "01":
                    x = list(filter(lambda v: 800<v<1500, x))
                elif "".join(str(anchor_to_plot).split(":")[-2:]) == "02":
                    x = list(filter(lambda v: 1000<v<1500, x))
                elif "".join(str(anchor_to_plot).split(":")[-2:]) == "03":
                    x = list(filter(lambda v: 500<v<1000, x))

                c = Counter(x)
                numbers = sorted(c.keys())

            # the histogram of the data
            axs[i].bar(numbers, [c[n] for n in numbers], facecolor=self.MEASURE_COLOUR, alpha=0.75, label='Mesure obtenue')

            axs[i].set_xlabel('Temps de propagation [UNITÉ ?]')
            axs[i].set_ylabel('Nombre d\'apparitions')
            axs[i].set_title(f'Temps de propagation du tag avec l\'ancre {"".join(str(anchor_to_plot).split(":")[-2:])}')
            axs[i].grid(True)
            if expected_mean is not None:
                axs[i].axvline(expected_mean, color='k', linestyle='dashed', linewidth=1, label='Coordonnée réele')
            
            axs[i].legend(loc='upper left')
            i += 1

        plt.show()

        



class GeolocationPlotter(Plotter):

    def __init__(self, datafile: str, _write: bool = True):
        super().__init__(datafile, ['anchor', 'x', 'y'], _write)

    def write(self, anchor: IPv6Address, coordinates: Coordinates):
        return super().write(["".join(str(anchor).split(":")[-2:]), coordinates.x, coordinates.y])
    
    
    def plot(self, anchor_to_plot: IPv6Address, expected_mean: Coordinates = None):
        # plot two histograms side by side
        x = []
        y = []

        with open(self.datafile) as f:
            reader = csv.DictReader(f)
            for row in reader:
                x.append(float(row['x']))
                y.append(float(row['y']))
        
        # Remove outliers            
        x = list(filter(lambda v: 11<v<11.5, x))
        y = list(filter(lambda v: 8<v<8.8, y))
        
        # side by side histograms
        fig, axs = plt.subplots(2, 1)
        fig.subplots_adjust(hspace=0.5)

        axs[0].hist(x, self.NUM_BINS, density=True, facecolor=self.ESTIMATE_COLOUR, alpha=0.75, label="Estimation via multilatération")
        axs[0].set_title(f'Coordonnées du tag {"".join(str(anchor_to_plot).split(":")[-2:])} pour l\'axe X obtenue par multilatération')
        axs[0].set_xlabel('Coordonnée X [mètres]')
        axs[0].set_ylabel('Frequency')
        if expected_mean is not None:
            axs[0].axvline(expected_mean.x, color=self.MEASURE_COLOUR, linestyle='dashed', linewidth=1, label='Coordonnée réele')
        axs[0].legend(loc='upper right')

        axs[1].hist(y, self.NUM_BINS, density=True, facecolor=self.ESTIMATE_COLOUR, alpha=0.75, label="Estimation via multilatération")
        axs[1].set_title(f'Coordonnées du tag {"".join(str(anchor_to_plot).split(":")[-2:])} pour l\'axe Y obtenue par multilatération')
        axs[1].set_xlabel('Coordonnée Y [mètres]')
        axs[1].set_ylabel('Frequency')
        if expected_mean is not None:
            axs[1].axvline(expected_mean.y, color=self.MEASURE_COLOUR, linestyle='dashed', linewidth=1, label='Coordonnée réele')
        axs[1].legend(loc='upper right')
        
        plt.show()

    
    def plot_xy(self, anchor_to_plot: IPv6Address, expected_mean: Coordinates = None):
        x = []
        y = []

        with open(self.datafile) as f:
            reader = csv.DictReader(f)
            for row in reader:
                x_val = float(row['x'])
                y_val = float(row['y'])
                if 11<x_val<11.5 and 8<y_val<8.8:   # Remove outliers
                    x.append(x_val)
                    y.append(y_val)
        
        # scatter plot
        plt.scatter(x, y, c=self.ESTIMATE_COLOUR ,alpha=0.25, label="Estimation via multilatération")
        plt.xlabel('Coordonnée X [mètres]')
        plt.ylabel('Coordonnée Y [mètres]')
        if expected_mean is not None:
            plt.scatter(expected_mean.x, expected_mean.y, c=self.MEASURE_COLOUR, alpha=1, s=200, label="Position réele")
        plt.legend(loc='upper left')
        plt.show()
    
    def plot_dist(self, anchor_to_plot: IPv6Address, expected_mean: Coordinates = None):
        x = []
        y = []

        with open(self.datafile) as f:
            reader = csv.DictReader(f)
            for row in reader:
                x_val = float(row['x'])
                y_val = float(row['y'])
                if 11<x_val<11.5 and 8<y_val<8.8:   # Remove outliers
                    x.append(abs(x_val - expected_mean.x))
                    y.append(abs(y_val - expected_mean.y))
        
        # side by side histograms
        fig, axs = plt.subplots(1, 2)
        fig.subplots_adjust(hspace=0.5)


        axs[0].hist(x, len(x), density=True, facecolor=self.ESTIMATE_COLOUR, alpha=0.75, histtype='step', cumulative=True)
        axs[0].set_xlabel("Erreur en X par rapport à la position réelle [mètres]")
        axs[0].set_ylabel("Probabilité d'occurrence")
        # axs[0].axvline(expected_mean.x, color=self.MEASURE_COLOUR, linestyle='dashed', linewidth=1, label='Coordonnée X réele')
        

        axs[1].hist(y, len(y), density=True, facecolor=self.ESTIMATE_COLOUR, alpha=0.75, histtype='step', cumulative=True)
        axs[1].set_xlabel("Erreur en Y par rapport à la potition réele [mètres]")
        axs[1].set_ylabel("Probabilité d'occurrence")
        # axs[1].axvline(expected_mean.x, color=self.MEASURE_COLOUR, linestyle='dashed', linewidth=1, label='Coordonnée Y réele')

        plt.legend(loc='upper left')

        plt.show()

if __name__ == "__main__":
    a = IPv6Address("fe80000000000000fdffffffffff0001")
    
    
    plotter = PropagationTimePlotter("test.csv")
    plotter.write(a, 5)
    plotter.write(a, 5)
    plotter.write(a, 6)
    plotter.write(a, 7)
    plotter.write(a, 6)
    plotter.close()

    plotter = PropagationTimePlotter("test.csv", False)
    plotter.plot(a, expected_mean=6)

    plotter = GeolocationPlotter("test.csv")
    plotter.write(a, Coordinates(4.1, 5.1))
    plotter.write(a, Coordinates(4.2, 5.2))
    plotter.write(a, Coordinates(4.3, 5.3))
    plotter.write(a, Coordinates(4.4, 5.4))
    plotter.close()

    plotter = GeolocationPlotter("test.csv", False)
    plotter.plot(a, expected_mean=Coordinates(4.2, 5.2))
