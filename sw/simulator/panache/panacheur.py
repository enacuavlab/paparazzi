from netCDF4 import Dataset
from scipy.interpolate import RegularGridInterpolator
import numpy as np
import time

NO_DATA = -99


class NoData(Exception):
    def __init__(self, true_coord):
        super().__init__("No data for true coordinates {}".format(true_coord))


class UnknownVariable(Exception):
    def __init__(self, var_name):
        super().__init__("Unknown variable '{}'".format(var_name))


class EndOfTime(Exception):
    def __init__(self):
        super().__init__("End of time!")


class Panacheur:
    def __init__(self, spray_file, swift_file, time_offset=None, time_scale=1, y_offset=0, y_scale=1, x_offset=0, x_scale=1, z_offset=0, z_scale=1):
        #print(time_offset, time_scale)
        self.spray = Dataset(spray_file, "r")
        self.swift = Dataset(swift_file, "r")
        self.interpolators = {}
        self.interpolators["PM10"] = self.get_spray_interpolator("PM10")
        self.interpolators["U"] = self.get_swift_interpolator("U")
        self.interpolators["V"] = self.get_swift_interpolator("V")
        self.interpolators["W"] = self.get_swift_interpolator("W")
        
        if time_offset is None:
            time_offset = time.time() - self.spray.variables["time"][0]
        
        self.mins = np.array([
            self.spray.variables["time"][0],
            self.spray.variables["Z"][0,0,0],
            self.spray.variables["y"][0],
            self.spray.variables["x"][0],
        ])
        self.offsets = np.array([time_offset, z_offset, y_offset, x_offset])
        self.scales = np.array([time_scale, z_scale, y_scale, x_scale])

        #print(f"mins: {self.mins}")

    def get_spray_interpolator(self, var):
        x = np.asarray(self.spray.variables["x"])
        y = np.asarray(self.spray.variables["y"])
        # Z: take a vertical slice
        z = np.asarray(self.spray.variables["Z"][:,0,0])
        # time: the last one is the same as the previous
        t = np.asarray(self.spray.variables["time"][:-1])
        # help(self.spray.variables[var])
        # remove last time
        return RegularGridInterpolator((t, z, y, x), self.spray.variables[var][:-1,:,:,:])
    
    def get_swift_interpolator(self, var):
        x = np.asarray(self.swift.variables["x"])
        y = np.asarray(self.swift.variables["y"])
        z = np.asarray(self.swift.variables["z"])
        t = np.asarray(self.swift.variables["time"])
        return RegularGridInterpolator((t, z, y, x), self.swift.variables[var])
    
    def transform_coords(self, coord):
        true_coord = (coord - self.mins - self.offsets) * self.scales + self.mins
        return true_coord

    def get_true_sample(self, interp, t, z, y, x):
        return interp([t, z, y, x])

    def get_sample(self, v: str, coord):
        """
        v: name of the variable
        """
        if v in self.interpolators:
            true_coord = self.transform_coords(coord)
            #print(true_coord)
            return self.interpolators[v](true_coord)
        else:
            raise UnknownVariable(v)
    
    def get_sample_safe(self, interp, coord):
        if self.is_in_domain(coord):
            return self.get_sample(interp, coord)
        else:
            true_coord = self.transform_coords(coord)
            if true_coord[0] > self.spray.variables["time"][-1]:
                raise EndOfTime()
            raise NoData(true_coord)
            #raise Exception("no data for true coordinates {}".format(true_coord))
    
    def get_vars_safe(self, variables, coord):
        return list(map(lambda v: self.get_sample_safe(v, coord), variables))

    def is_in_domain(self, coord):
        domain = self.get_true_domain()
        true_coord = self.transform_coords(coord)
        return np.all(domain[:, 0] <= true_coord) and np.all(domain[:, 1] >= true_coord)
    
    @staticmethod
    def make_coordinates(t, z, y, x):
        return np.array([t, z, y, x])
   
    def get_true_domain(self):
        return np.array([
        [np.min(self.spray.variables["time"]), np.max(self.spray.variables["time"])],
        [np.min(self.spray.variables["Z"][:,0,0]), np.max(self.spray.variables["Z"][:,0,0])],
        [np.min(self.spray.variables["y"]), np.max(self.spray.variables["y"])],
        [np.min(self.spray.variables["x"]), np.max(self.spray.variables["x"])]
        ])
    
    def get_domain(self):
        domain = self.get_true_domain()
        return domain / self.scales[:, np.newaxis] - self.offsets[:, np.newaxis]


if __name__ == "__main__":
    spray_file = "spray.bin_n1_d1_t1.nc"
    swift_file = "swift_merged.nc"
    p = Panacheur(spray_file, swift_file)
    
    print("domain:", p.get_true_domain())
    start = time.time()
    for i in range(100):
        coords = (time.time()+3600, 500, 6930676+5*i, 561060)
        pm10, u, v, w = p.get_vars_safe(("PM10", "U", "V", "W"), coords)
        print(f"pm10: {pm10}\tu:{u}\tv:{v}\tw:{w}")
    dt = time.time() - start
    print(f"dt={dt}")

  
  
  
