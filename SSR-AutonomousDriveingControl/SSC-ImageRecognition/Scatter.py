import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time

#https://github.com/shtsno24/Fast_Render_Matplotlib
class Scatter:

    def __init__(self, fig, ax, plot_area=(1000, 1000), len_points=100, show_icon=False, icon_radius=100, PySimpleGUI=False):
        self.fig = fig
        self.plot_area = plot_area
        self.icon_radius = icon_radius if show_icon is True else None
        self.orientation = 0.0

        # axis setup
        self.pos_ax = ax
        self.pos_ax.set_xlim(-plot_area[0], plot_area[0])
        self.pos_ax.set_ylim(-plot_area[1], plot_area[1])
        self.pos_points = self.pos_ax.scatter([], [])
        self.xy = [[0.0, 0.0] for x in range(len_points)]
        if show_icon is True:
            self.agent_icon = mpatches.RegularPolygon(xy=(0, 0), numVertices=4, radius=self.icon_radius, orientation=0.0, ec="r", fill=False)
            self.pos_ax.add_patch(self.agent_icon)

        # show figure
        if PySimpleGUI is False:
            self.fig.canvas.draw()
            self.fig.show()

    def update_data(self, points):
        # draw background with white
        self.pos_ax.draw_artist(self.pos_ax.patch)

        # plot points
        self.xy = list(zip(*points))
        self.pos_points.set_offsets(self.xy)
        self.pos_ax.draw_artist(self.pos_points)

        # plot the icon
        if self.icon_radius is not None:
            self.agent_icon.xy = self.xy[0]
            self.agent_icon.orientation = self.orientation
            self.pos_ax.draw_artist(self.agent_icon)

        # update this graph
        self.fig.canvas.blit(self.pos_ax.bbox)

    def plot(self, points, orientation=0.0):
        self.orientation = orientation
        self.update_data(points)
        self.fig.canvas.flush_events()

    def cla(self):
        # draw background with white
        self.pos_ax.draw_artist(self.pos_ax.patch)

        # plot points
        self.xy = [[0.0, 0.0] for x in range(len(self.xy))]
        self.pos_points.set_offsets(self.xy)
        self.pos_ax.draw_artist(self.pos_points)

        # plot the icon
        if self.icon_radius is not None:
            self.agent_icon.xy = self.xy[0]
            self.agent_icon.orientation = 0.0
            self.pos_ax.draw_artist(self.agent_icon)

        # update this graph
        self.fig.canvas.blit(self.pos_ax.bbox)
        self.fig.canvas.flush_events()