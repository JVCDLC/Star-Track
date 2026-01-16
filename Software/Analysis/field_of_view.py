"""
This script evaluates the angular size of celestial bodies and compares it with
the effective field of view of a telescope. The results are used to assess
framing margins and to determine the pointing accuracy required for precise
telescope alignment on astronomical targets.
"""


import numpy as np


class CelestialBody:
    def __init__(self, name: str, diameter_km: float, distance_from_earth_km: float):
        self.name: str = name
        self.diameter: float = diameter_km
        self.distance_from_earth: float = distance_from_earth_km
        self.field_of_view_deg: float = 2 * np.rad2deg(np.arctan2((self.diameter / 2), self.distance_from_earth))
    def __str__(self):
        return (f"{self.name} has a diameter of {self.diameter} km\n"
                f"and is a distance from earth to {self.distance_from_earth} km\n"
                f"which corresponds to a field of view of {self.field_of_view_deg} deg\n.")
    def print_field_of_view_deg(self):
        print(f"{self.name}: {self.field_of_view_deg} deg")


class Telescope:
    def __init__(self, focal_length_mm: float, eye_piece_focal_length_mm: float, eye_piece_field_of_view_deg: float):
        self.focal_length_mm: float = focal_length_mm
        self.eye_piece_focal_length_mm: float = eye_piece_focal_length_mm
        self.eye_piece_field_of_view_deg: float = eye_piece_field_of_view_deg
        self.magnification: float = self.focal_length_mm / self.eye_piece_focal_length_mm
        self.field_of_view_deg: float = self.eye_piece_field_of_view_deg / self.magnification
    def __str__(self):
        return (f"A telescope with a focal length of {self.focal_length_mm} mm,\n"
                f"an eye piece focal length of {self.eye_piece_focal_length_mm} mm\n"
                f"and an eye piece field of view of {self.field_of_view_deg} deg \n"
                f"has a magnification of x{self.magnification}\n"
                f"and a field of view of {self.field_of_view_deg} deg\n.")
    def get_celestial_body_proportion(self, celestial_body: CelestialBody) -> float:
        return celestial_body.field_of_view_deg / self.field_of_view_deg
    def print_celestial_body_proportion(self, celestial_body: CelestialBody):
        print(f"{celestial_body.name} would take about {100 * self.get_celestial_body_proportion(celestial_body)}% of the image width.")
    def get_count_of_celestial_body(self, celestial_body: CelestialBody) -> float:
        return self.field_of_view_deg / celestial_body.field_of_view_deg
    def print_count_of_celestial_body(self, celestial_body: CelestialBody):
        print(f"{celestial_body.name} could enter {self.get_count_of_celestial_body(celestial_body)} times in the image.")
    def print_celestial_body_view(self, celestial_body: CelestialBody):
        self.print_celestial_body_proportion(celestial_body)
        self.print_count_of_celestial_body(celestial_body)


def main():
    TELESCOPE: Telescope = Telescope(400, 25, 50)
    SUN: CelestialBody = CelestialBody("Sun", 1_390_000, 150_000_000)
    MOON: CelestialBody = CelestialBody("Moon", 3_475, 384_400)
    JUPITER: CelestialBody = CelestialBody("Jupiter", 139_820, 800_000_000)
    VENUS: CelestialBody = CelestialBody("Venus", 12_104, 108_000_000)
    MERCURY: CelestialBody = CelestialBody("Mercury", 4_879, 77_000_000)
    URANUS: CelestialBody = CelestialBody("Uranus", 50_724, 2_800_000_000)

    TELESCOPE.print_celestial_body_view(MOON)


if __name__ == "__main__":
    main()
