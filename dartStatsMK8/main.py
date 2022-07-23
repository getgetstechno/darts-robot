# MK8 updating MK6 : using all MK7 but displaying in 2 different classes

from kivy.uix.floatlayout import FloatLayout

from kivy.app import App
from kivy.graphics import Color, Ellipse
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg

import pandas
import time
import os
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats

# variables for excel
data1 = []
data2 = []
data3 = []
ts = time.time()
OutF = r'C:\Users\Ik\Documents\Pycharm projects\gameDev\dartStatsMK6\dartStatsMK6\OUTPUTS SVG'

x_values1 = []
y_values1 = []
angles1 = []
left_distances1 = []

x_values2 = []
y_values2 = []
angles2 = []
left_distances2 = []

x_values3 = []
y_values3 = []
angles3 = []
left_distances3 = []

slope1 = 0
slope2 = 0
intercept1 = 0
intercept2 = 0
r1 = 0
r2 = 0

slope3 = 0
slope4 = 0
intercept3 = 0
intercept4 = 0
r3 = 0
r4 = 0

slope5 = 0
slope6 = 0
intercept5 = 0
intercept6 = 0
r5 = 0
r6 = 0


def truncate(n):
    return int(n * 1000) / 1000


class FirstWindow(Screen):
    cursor_state = False
    color1 = False

    def on_touch_down(self, touch):

        print(str(touch.x) + " " + str(touch.y))
        if self.cursor_state and 620 < touch.x < 1550:
            with self.canvas:
                if self.color1:
                    Color(1, .5, 0)
                else:
                    Color(1, 0, 0)
                d = 10.
                Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))
                self.ids.x_value.text = str((touch.x - (7 * self.width) / 10) / 18.4)
                self.ids.y_value.text = str((touch.y - (self.height / 2)) / 18.25)
            self.cursor_state = False
            self.color1 = False
            button = self.ids.add_point_btn
            button.background_color = (0, 0, 1, 0.2)
        return super(FirstWindow, self).on_touch_down(touch)

    def save_point(self):

        angle = int(float(self.ids.angle.text))
        fd = self.ids.front_distance.text
        ld = int(float(self.ids.left_distance.text))
        h = self.ids.height.text
        xas = int(float(self.ids.x_value.text))
        yas = int(float(self.ids.y_value.text))

        canon = int(self.ids.canon.text)

        if canon == 1:
            data1.append([angle, fd, ld, h, xas, yas])
            df = pandas.DataFrame(data1, columns=['angle', 'fd', 'ld', 'h', 'x', 'y'])
            df.to_csv(os.path.join(OutF, 'data1_' + str(ts) + '.csv'))
            print(data1)
            print("x : " + str(data1[0]))
            x_values1.append(xas)
            y_values1.append(yas)
            angles1.append(angle)
            left_distances1.append(ld)

        elif canon == 2:
            data2.append([angle, fd, ld, h, xas, yas])
            df = pandas.DataFrame(data1, columns=['angle', 'fd', 'ld', 'h', 'x', 'y'])
            df.to_csv(os.path.join(OutF, 'data2_' + str(ts) + '.csv'))
            print(data2)
            print("x : " + str(data2[0]))
            x_values2.append(xas)
            y_values2.append(yas)
            angles2.append(angle)
            left_distances2.append(ld)

        elif canon == 3:
            data3.append([angle, fd, ld, h, xas, yas])
            df = pandas.DataFrame(data1, columns=['angle', 'fd', 'ld', 'h', 'x', 'y'])
            df.to_csv(os.path.join(OutF, 'data3_' + str(ts) + '.csv'))
            print(data3)
            print("x : " + str(data3[0]))
            x_values3.append(xas)
            y_values3.append(yas)
            angles3.append(angle)
            left_distances3.append(ld)

        if canon >= 3:
            canon = 1
        else:
            canon += 1

        self.ids.canon.text = str(canon)

    def add_point(self):
        self.cursor_state = True
        button = self.ids.add_point_btn
        button.background_color = (1, 0, 0, 1)

    def select_objective(self):
        print("hello world haha ")
        self.cursor_state = True
        self.color1 = True

    def calculate_obj(self):

        x_as = int(float(self.ids.x_value.text))
        y_as = int(float(self.ids.y_value.text))
        print("x_as : " + str(x_as))
        print("y as : " + str(y_as))

        # fill x_as in function to determine distance needed !!!
        # formule <-> y = s1*x_as + i1
        obj_dist1 = slope1 * x_as + intercept1
        print("distance to use for objective (cannon 1) : " + str(obj_dist1))
        # fill Y-as in function to determine angle needed !!!
        # formule <-> y = s2*y_as + i2
        obj_angle1 = slope2 * y_as + intercept2
        print("angle to use for objective (cannon 1) : " + str(obj_angle1))

        obj_dist2 = slope3 * x_as + intercept3
        print("distance to use for objective with cannon 2 : " + str(obj_dist2))
        obj_angle2 = slope4 * y_as + intercept4
        print("angle to use for objective (cannon 2) : " + str(obj_angle2))

        obj_dist3 = slope5 * x_as + intercept5
        print("distance to use for objective with cannon 3 : " + str(obj_dist3))
        obj_angle3 = slope6 * y_as + intercept6
        print("angle to use for objective (cannon 3) : " + str(obj_angle3))

        # add obj_angle and obj_dist inside the label
        self.ids.objective_label1.text = f'           1 \n a : {truncate(obj_dist1)}, d : {truncate(obj_angle1)}'
        self.ids.objective_label2.text = f'           2 \n a : {truncate(obj_dist2)}, d : {truncate(obj_angle2)}'
        self.ids.objective_label3.text = f'           3 \n a : {truncate(obj_dist3)}, d : {truncate(obj_angle3)}'


class SecondWindow(Screen, FloatLayout):

    def show_graph(self):
        # part one for canon 1 ==========================================================

        x1 = np.array(x_values1)
        y1 = np.array(y_values1)
        angle1 = np.array(angles1)
        left_dis1 = np.array(left_distances1)

        # first graph, x = x-values, y = left distances
        s1, i1, r_1, p1, stderr1 = scipy.stats.linregress(x1, left_dis1)

        global slope1
        slope1 = s1
        global intercept1
        intercept1 = i1
        global r1
        r1 = r_1

        line1 = f'Regression line: y={intercept1:.2f}+{slope1:.2f}x, r={r1:.2f}'
        fig1, ax1 = plt.subplots()
        ax1.plot(x1, left_dis1, linewidth=0, marker='s', label='Data points')
        ax1.plot(x1, intercept1 + slope1 * x1, label=line1)
        ax1.set_xlabel('x-values')
        ax1.set_ylabel('left distances')
        ax1.legend(facecolor='white')
        print("slope : " + str(slope1) + " i : " + str(intercept1) + " r : " + str(r1))
        box1 = self.ids.plot_layout1
        box1.add_widget(FigureCanvasKivyAgg(plt.gcf()))

        # second graph, x = y-values, y = angles

        s2, i2, r_2, p2, stderr2 = scipy.stats.linregress(y1, angle1)

        global slope2
        slope2 = s2
        global intercept2
        intercept2 = i2
        global r2
        r2 = r_2

        line2 = f'Regression line: y={intercept2:.2f}+{slope2:.2f}x, r={r2:.2f}'
        fig2, ax2 = plt.subplots()
        ax2.plot(y1, angle1, linewidth=0, marker='s', label='Data points')
        ax2.plot(y1, intercept2 + slope2 * y1, label=line2)
        ax2.set_xlabel('y-values')
        ax2.set_ylabel('angle')
        ax2.legend(facecolor='white')
        print("slope : " + str(slope2) + " i : " + str(intercept2) + " r : " + str(r2))
        box2 = self.ids.plot_layout4
        box2.add_widget(FigureCanvasKivyAgg(plt.gcf()))

        # part 2 for cannon 2 ============================================================

        x2 = np.array(x_values2)
        y2 = np.array(y_values2)
        angle2 = np.array(angles2)
        left_dis2 = np.array(left_distances2)

        # first graph, x = x-values, y = left distances
        s3, i3, r_3, p3, stderr3 = scipy.stats.linregress(x2, left_dis2)

        global slope3
        slope3 = s3
        global intercept3
        intercept3 = i3
        global r3
        r3 = r_3

        line3 = f'Regression line: y={intercept3:.2f}+{slope3:.2f}x, r={r3:.2f}'
        fig3, ax3 = plt.subplots()
        ax3.plot(x2, left_dis2, linewidth=0, marker='s', label='Data points')
        ax3.plot(x2, intercept3 + slope3 * x2, label=line3)
        ax3.set_xlabel('x-values')
        ax3.set_ylabel('left distances')
        ax3.legend(facecolor='white')
        print("slope : " + str(slope3) + " i : " + str(intercept3) + " r : " + str(r3))
        box3 = self.ids.plot_layout2
        box3.add_widget(FigureCanvasKivyAgg(plt.gcf()))

        # second graph, x = y-values, y = angles

        s4, i4, r_4, p4, stderr4 = scipy.stats.linregress(y2, angle2)

        global slope4
        slope4 = s4
        global intercept4
        intercept4 = i4
        global r4
        r4 = r_4

        line4 = f'Regression line: y={intercept4:.2f}+{slope4:.2f}x, r={r4:.2f}'
        fig4, ax4 = plt.subplots()
        ax4.plot(y2, angle2, linewidth=0, marker='s', label='Data points')
        ax4.plot(y2, intercept4 + slope4 * y2, label=line4)
        ax4.set_xlabel('y-values')
        ax4.set_ylabel('angle')
        ax4.legend(facecolor='white')
        print("slope : " + str(slope4) + " i : " + str(intercept4) + " r : " + str(r4))
        box4 = self.ids.plot_layout5
        box4.add_widget(FigureCanvasKivyAgg(plt.gcf()))

        # part 3 for cannon 3 ============================================================

        x3 = np.array(x_values3)
        y3 = np.array(y_values3)
        angle3 = np.array(angles3)
        left_dis3 = np.array(left_distances3)

        # first graph, x = x-values, y = left distances
        s5, i5, r_5, p5, stderr5 = scipy.stats.linregress(x2, left_dis2)

        global slope5
        slope5 = s5
        global intercept5
        intercept5 = i5
        global r5
        r5 = r_5

        line5 = f'Regression line: y={intercept5:.2f}+{slope5:.2f}x, r={r5:.2f}'
        fig5, ax5 = plt.subplots()
        ax5.plot(x3, left_dis3, linewidth=0, marker='s', label='Data points')
        ax5.plot(x3, intercept5 + slope5 * x3, label=line5)
        ax5.set_xlabel('x-values')
        ax5.set_ylabel('left distances')
        ax5.legend(facecolor='white')
        print("slope : " + str(slope5) + " i : " + str(intercept5) + " r : " + str(r5))
        box5 = self.ids.plot_layout3
        box5.add_widget(FigureCanvasKivyAgg(plt.gcf()))

        # second graph, x = y-values, y = angles

        s6, i6, r_6, p6, stderr6 = scipy.stats.linregress(y3, angle3)

        global slope6
        slope6 = s6
        global intercept6
        intercept6 = i6
        global r6
        r6 = r_6

        line6 = f'Regression line: y={intercept6:.2f}+{slope6:.2f}x, r={r6:.2f}'
        fig6, ax6 = plt.subplots()
        ax6.plot(y3, angle3, linewidth=0, marker='s', label='Data points')
        ax6.plot(y3, intercept6 + slope6 * y3, label=line6)
        ax6.set_xlabel('y-values')
        ax6.set_ylabel('angle')
        ax6.legend(facecolor='white')
        print("slope : " + str(slope6) + " i : " + str(intercept6) + " r : " + str(r6))
        box6 = self.ids.plot_layout6
        box6.add_widget(FigureCanvasKivyAgg(plt.gcf()))


class WindowManager(ScreenManager):
    pass


class ControllerApp(App):

    def build(self):
        return Builder.load_file("Controller.kv")


if __name__ == '__main__':
    ControllerApp().run()
