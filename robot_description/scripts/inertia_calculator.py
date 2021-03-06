#!/usr/bin/env python
"""Based on https://github.com/artur84/my_mira_description/blob/a4f6e47fed05aafc5637906dbfb4157033ddc68f/scripts/inertia_calculator.py
"""

from __future__ import print_function


class InertialCalculator(object):

    def __init__(self):
        print("InertialCalculator Initialised...")

    def start_ask_loop(self):
        selection = "START"
        while selection != "Q":
            print("#############################")
            print("Select Geometry to Calculate:")
            print("[1]Box width(w)*depth(d)*height(h)")
            print("[2]Sphere radius(r)")
            print("[3]Cylinder radius(r)*height(h)")
            print("[Q]END program")
            selection = input(">>")
            self.select_action(selection)
        print("InertialCaluclator Quit...Thank you")

    def select_action(self, selection):
        if str(selection) == "1":
            mass = float(input("mass (kg) >> "))
            width = float(input("width (m) >> "))
            depth = float(input("depth (m) >> "))
            height = float(input("height (m) >> "))
            self.calculate_box_inertia(m=mass, w=width, d=depth, h=height)

        elif str(selection) == "2":
            mass = float(input("mass>>"))
            radius = float(input("radius>>"))
            self.calculate_sphere_inertia(m=mass, r=radius)

        elif str(selection) == "3":
            mass = float(input("mass>>"))
            radius = float(input("radius>>"))
            height = float(input("height>>"))
            self.calculate_cylinder_inertia(m=mass, r=radius, h=height)

        elif str(selection) == "Q":
            print("Selected Quit")

        else:
            print("Usage: Select one of the give options")

    @staticmethod
    def calculate_box_inertia(m, w, d, h):
        Iw = (m / 12.0) * (pow(d, 2) + pow(h, 2))
        Id = (m / 12.0) * (pow(w, 2) + pow(h, 2))
        Ih = (m / 12.0) * (pow(w, 2) + pow(d, 2))
        print("BOX w*d*h, Iw = " + \
            str(Iw) + ",Id = " + str(Id) + ",Ih = " + str(Ih))

    @staticmethod
    def calculate_sphere_inertia(m, r):
        I = (2 * m * pow(r, 2)) / 5.0
        print("SPHERE Ix,y,z = " + str(I))

    @staticmethod
    def calculate_cylinder_inertia(m, r, h):
        Ix = (m / 12.0) * (3 * pow(r, 2) + pow(h, 2))
        Iz = (m * pow(r, 2)) / 2.0
        print("Cylinder Ix,y = " + str(Ix) + ",Iz = " + str(Iz))


if __name__ == "__main__":
    inertial_object = InertialCalculator()
    inertial_object.start_ask_loop()
