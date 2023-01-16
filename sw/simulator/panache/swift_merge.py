from netCDF4 import Dataset
import numpy as np


swift_files = [
    "swift.bin_2019-09-26_01-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_02-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_03-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_04-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_05-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_06-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_07-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_08-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_09-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_10-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_11-00-00_n1_d1_t1.nc",
    "swift.bin_2019-09-26_12-00-00_n1_d1_t1.nc"]


dims = ['x', 'y', 'Z']
toinclude = ['x', 'y', 'Z']


def get_data(filename):
    d = Dataset(filename, "r")
    U = d["U"][0,:]
    V = d["V"][0,:]
    W = d["W"][0,:]
    t = d["time"][0]
    return t, U, V, W


def merge(filenames):
    data = map(get_data, filenames)
    tl, ul, vl, wl = zip(*data)
    t = np.array(tl)
    u = np.array(ul)
    v = np.array(vl)
    w = np.array(wl)
    return t, u, v, w

def make_dataset(filename):
    # use first file as model for attributes, x, y, z
    with Dataset(filename, "w") as dst, Dataset(swift_files[0], "r") as src:
        # copy attributes
        for name in src.ncattrs():
            dst.setncattr(name, src.getncattr(name))
        
        # copy dimensions
        for name, dimension in src.dimensions.items():
            if name in dims:
                dst.createDimension( name, len(dimension))
                
        # copy all file data for variables that are included in the toinclude list
        for name, variable in src.variables.items():
            if name in toinclude:
                x = dst.createVariable(name, variable.datatype, variable.dimensions)
                dst.variables[name][:] = src.variables[name][:]
        
        dst.createDimension("z")
        dst.createVariable("z", "f4", ("z",))
        dst["z"][:] = np.array([level.mean() for level in src.variables["Z"]])
        
        dst.createDimension("time")
        dst.createVariable("time", "f8", ("time",))
        
        t, u, v, w = merge(swift_files)
        dst["time"][:] = t
        
        
        dst.createVariable("U", "f4", ("time", "z", "y", "x"))
        dst.createVariable("V", "f4", ("time", "z", "y", "x"))
        dst.createVariable("W", "f4", ("time", "z", "y", "x"))
        
        dst["U"][:] = u
        dst["V"][:] = v
        dst["W"][:] = w
       
if __name__ == '__main__':
    make_dataset('swift_merged.nc') 

