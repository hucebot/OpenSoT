import viser
import numpy as np

class plot:
    def __init__(self, title, size, legend_label, server, dt, max_len=1000):
        self.size = size
        self.max_len = max_len
        self.dt = dt

        # One shared time axis
        self.x = self.dt * np.arange(max_len, dtype=np.float64)

        # Y series: one array per joint
        self.ys = [np.zeros(max_len, dtype=float) for _ in range(size)]

        # uPlot data format: (x, y0, y1, ..., yN)
        self.data = (self.x, *self.ys)

        # Series definition: one series per array in data

        self.series = (
            viser.uplot.Series(label="t"),
            *[viser.uplot.Series(label=f"{legend_label}{i}", stroke=self.make_color(i, size), width=2) for i in range(size)])

        self.plot_handle = server.gui.add_uplot(
            data=self.data,
            series=self.series,
            title=title,
            scales={"x": viser.uplot.Scale(time=False, auto=True), "y": viser.uplot.Scale(auto=True)},
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )

    def make_color(self, i, n):
        # HSV equally spaced around the hue circle
        h = i / n
        s = 0.65
        v = 0.85
        import colorsys
        r, g, b = colorsys.hsv_to_rgb(h, s, v)
        return f"rgb({int(r * 255)}, {int(g * 255)}, {int(b * 255)})"

    def update(self, new_data):
        # Append new data
        self.x = np.roll(self.x, -1)
        self.x[-1] = self.x[-2] + self.dt

        for i in range(self.size):
            self.ys[i] = np.roll(self.ys[i], -1)
            self.ys[i][-1] = new_data[i]

        self.plot_handle.data = (self.x.tolist(), *[y.tolist() for y in self.ys])
