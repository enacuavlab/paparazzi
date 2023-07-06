from __future__ import annotations

import typing
import json
import os,io
import pickle
import warnings
import abc

from dataclasses import dataclass

#################### Format check ####################

def assert_TopoJSON(json_obj) -> None:
    assert isinstance(json_obj,dict)
    assert json_obj["type"] == "Topology"
    assert isinstance(json_obj["arcs"],list)
    assert isinstance(json_obj["objects"],dict)
    
def test_TopoJSON(json_obj) -> bool:
    try:
        assert_TopoJSON(json_obj)
        return True
    except AssertionError:
        return False
    except KeyError:
        return False
    
def assert_GeometryCollection(json_obj) -> None:
    assert isinstance(json_obj,dict)
    assert json_obj["type"] == "GeometryCollection"
    assert isinstance(json_obj["geometries"],list)
    
def test_GeometryCollection(json_obj) -> bool:
    try:
        assert_GeometryCollection(json_obj)
        return True
    except AssertionError:
        return False
    except KeyError:
        return False

#################### Base dataclasses ####################

@dataclass(order=False)
class SamplePoint():
    """
    Generic dataclass for 4D (spatial + time) sample points.
    
    Note: The coordinate system used is prescibed by the user, not the class.
    That is, [x,y,z] may be [Easting,Northing,Height] or [Longitude,Latitude,Altitude],
    or whatever else...
    
    Similarly, `time` can be in seconds from 0, from Epoch ... 
    """
    
    __slots__ = ("x","y","z","time","val")
    x:float
    y:float
    z:float
    time:float
    val:object
    
    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)
        self.time = float(self.time)
    
    def to_TopoJSON_dict(self) -> typing.Dict[str,typing.Any]:
        """
        Output a JSON-like dictionary, such that is `json.dump` is called on it,
        one obtains a correct GeoJSON "Point" with three coordinates [x,y,z], and
        two additional properties, "time" and "val". 
        """
        output = dict()
        output["type"] = "Point"
        output["properties"] = {"time":self.time,"val":self.val}
        output["coordinates"] = [self.x,self.y,self.z]
        return output        
    
    @staticmethod
    def from_TopoJSON_dict(d:dict) -> SamplePoint:
        """
        Parse a SamplePoint from a JSON-like dictionary, formatted as a GeoJSON "Point" with
        three coordinates [x,y,z], and two additional properties, "time" and "val". 
        """
        assert d["type"] == "Point"
        t = d["properties"]["time"]
        val = d["properties"]["val"]
        x = d["coordinates"][0]
        y = d["coordinates"][1]
        z = d["coordinates"][2]
        
        return SamplePoint(x,y,z,t,val)
        
@dataclass(order=False)
class ScalarSample(SamplePoint):
    """
    Specifies the SamplePoint class for having `val` be a float 
    """
    __slots__ = ()
    val:float
    
    def __post_init__(self):
        super().__post_init__()
        self.val = float(self.val)
    
    
#################### Parsers classes ####################

#---------- Abstract toplevel ----------# 

class SampleWriter(abc.ABC):
    @abc.abstractmethod
    def write(self,samplePoint:typing.Union[dict,SamplePoint]) -> None:
        """Ensure correct format before writing if given as dict"""
        ...
        
    @abc.abstractmethod
    def close(self) -> None:
        ...
        
class SampleReader(abc.ABC):
    @abc.abstractmethod
    def read(self) -> typing.Optional[SamplePoint]:
        """Return None once there is nothing left to read"""
        ...
        
    @abc.abstractmethod
    def close(self) -> None:
        ...

#---------- TopoJSON ----------# 

class SampleWriter_TopoJSON(SampleWriter):
    """
    Writer class to auto-format SamplePoint objects to TopoJSON (add correct toplevel format,
    see https://github.com/topojson/topojson-specification/blob/master/README.md#2-topojson-objects)
    """
    
    header="""
{
    "type": "Topology",
    "arcs": [],
    "objects": {
        "type": "GeometryCollection",
        "geometries": [
"""

    footer="""
        ]
    }
}
    """
    def __init__(self,f:typing.Union[os.PathLike,io.TextIOWrapper],autoclose:typing.Optional[bool]=None) -> None:
        """
        If f is:
        - a `TextIOWrapper`: write directly to it
        - a `PathLike`: open it, then write.
        
        If `autoclose` is None, the writer automatically closes the file at the object deletion, 
        is it was given as a PathLike. But it doesn't if it was a TextIOWrapper.
        
        If `autoclose` is a either True or False, then the wanted behavior is applied at object deletion,
        regardless of the initial nature of `f`.

        """
        
        if isinstance(f,io.TextIOWrapper):
            self._file = f
            assert self._file.writable()
            self._close = False
        else:
            self._file = open(f,mode='w')
            self._close = True
            
        if autoclose is not None:
            self._close = autoclose
            
        self._file.write(self.header)
        self._first_write = True
        
    def __exit__(self) -> None:
        if self._close:
            self.close()
        
    def write(self,samplePoint:typing.Union[dict,SamplePoint]) -> None:  
        """
        Write the given SamplePoint to the underlying file.
        
        If the object is a dictionary:
        - if `parseDict` is True, convert back and forth from SamplePoint 
        - otherwise, parse the dict to ensure correct format, then write the dictionary through `json.dump`
        """
          
        if not self._first_write:
            self._file.write(',\n')
        else:
            self._first_write = False
        
        if isinstance(samplePoint,SamplePoint):
            json.dump(samplePoint.to_TopoJSON_dict(),self._file)
        else:
            json.dump(SamplePoint.from_TopoJSON_dict(samplePoint).to_TopoJSON_dict(),self._file)
        
    def close(self) -> None:
        if not self._file.closed:
            self._file.write(self.footer)
            self._file.close()


class SampleReader_TopoJSON(SampleReader):
    """
    Read-only class to parse Samples from a TopoJSON encoding
    Parsed samples can be read from index or through iteration
    
    @Warning: Since it assumes JSON-like formating, it will open and load the whole file at once
    (may hurt memory performance)
    """
    
    def __init__(self,f) -> None:
        """
        If f is:
        - a `dict`: tries to parse it directly as if produced from reading a TopoJSON
        - a `TextIOWrapper`: read from it then parse (does not close the stream)
        - a `PathLike`: open the associated file, parse it, then close the file
        - anything else: iterate throught it, trying to parse everything it contains as a SamplePoint,
            assuming JSON-like dict format 
            
        """
        if isinstance(f,dict):
            self._samples = self._parse_dict(f)
        
        elif isinstance(f,io.TextIOWrapper):
                _file = f
                assert _file.readable()
                _json_obj = json.load(_file)
                self._samples = self._parse_dict(_json_obj)
                
        elif isinstance(f,os.PathLike) or isinstance(f,str):
                with open(f,mode='r') as _file:
                    _json_obj = json.load(_file)
                    self._samples = self._parse_dict(_json_obj) 
                    
        else:
            self._samples = [SamplePoint.from_TopoJSON_dict(o) for o in f]
            
    @staticmethod
    def _parse_dict(d:dict) -> typing.List[SamplePoint]:
        """
        Try to parse a dictionary (as produced from a JSON, 
        see https://docs.python.org/fr/3/library/json.html#json-to-py-table),
        assuming either toplevel TopoJSON, or "GeometryCollection" TopoJSON object 
        (see https://github.com/topojson/topojson-specification/blob/master/README.md#2-topojson-objects).
        
        It expects the "GeometryCollection" to contain a list of ONLY SamplePoint like objects
        (that is GeoJSON "Point" with 3 coordinates, and two properties, "time" and "val")
        """
        assert isinstance(d,dict)
        if test_TopoJSON(d):
            geometryCollection = d["objects"]
        else:
            geometryCollection = d
        assert_GeometryCollection(geometryCollection)
        
        json_sample_list = geometryCollection["geometries"]
            
        return [SamplePoint.from_TopoJSON_dict(o) for o in json_sample_list]
    
    def __getitem__(self,key) -> SamplePoint:
        return self._samples[key]
            
    def __iter__(self):
        return self._samples.__iter__()
    
    def read(self) -> typing.Optional[SamplePoint]:
        """
        Read through the iterator. Return None once the iterator's is reached
        """
        try:
            return self.__next__()
        except StopIteration:
            return None
        
    def close(self) -> None:
        pass
        
    
#---------- Python pickle ----------# 

class SampleWriter_pickle(SampleWriter):
    """
    Writer class using the Python pickle protocol to write SamplePoint in binary files
    (through a stream)
    """
    def __init__(self,f:typing.Union[os.PathLike,io.BytesIO],autoclose:typing.Optional[bool]=None) -> None:
        """
        If f is:
        - a `BytesIO`: write directly to it
        - a `PathLike`: open it, then write.
        
        If `autoclose` is None, the writer automatically closes the file at the object deletion
        if it was given as a PathLike. But it doesn't if it was a BytesIO.
        
        If `autoclose` is either True or False, then the wanted behavior is applied at object deletion,
        regardless of the initial nature of `f`.

        """
        
        if isinstance(f,io.BytesIO):
            self._file = f
            assert self._file.writable()
            self._close = False
        else:
            self._file = open(f,mode='wb')
            self._close = True
            
        if autoclose is not None:
            self._close = autoclose
            
    def __exit__(self) -> None:
        if self._close:
            self.close()
        
    def write(self,samplePoint:typing.Union[dict,SamplePoint]) -> None:    
        """
        Write the given SamplePoint to the underlying file. 
        
        If the input is a dictionary, parse it to a SamplePoint object first
        """
        
        if isinstance(samplePoint,SamplePoint):
            pickle.dump(samplePoint,self._file)
        else:
            pickle.dump(SamplePoint.from_TopoJSON_dict(samplePoint),self._file)

        
    def close(self) -> None:
        self._file.close()
        
class SampleReader_pickle(SampleReader):
    """
    Reader class using the Python pickle protocol to read SamplePoint from binary files
    
    Note: This class reads file through a stream. Depending on how it was pickled, data may be 
    recovered as a list or through repeated iterations. Because of this, access is provided only
    through iterator
    """
    def __init__(self,f:typing.Union[os.PathLike,io.BytesIO],autoclose:typing.Optional[bool]=None) -> None:
        """
        If f is:
        - a `BytesIO`: read directly to it
        - a `PathLike`: open it, then read.
        
        If `autoclose` is None, the reader automatically closes the file at the object deletion
        if it was given as a PathLike. But it doesn't if it was a BytesIO.
        
        If `autoclose` is either True or False, then the wanted behavior is applied at object deletion,
        regardless of the initial nature of `f`.
        """
        
        if isinstance(f,io.BytesIO):
            self._file = f
            assert self._file.readable()
            self._close = False
        else:
            self._file = open(f,mode='rb')
            self._close = True
            
        if autoclose is not None:
            self._close = autoclose
    
        self._pickled_list_it:typing.Optional[typing.Iterator] = None
            
    def __iter__(self):
        return self
    
    def __next__(self) -> SamplePoint:
        if self._pickled_list_it is not None:
            try:
                o = next(self._pickled_list_it)
                if not isinstance(o,SamplePoint):
                    warnings.warn(f"pickled list contains an unexpected object of type: {type(o)}")
                return o
            except StopIteration:
                self._pickled_list_it = None
        
        try:
            p = pickle.dump(self._file)
        except EOFError:
            raise StopIteration
        
        if isinstance(p,list):
            self._pickled_list_it =p.__iter__()
            return self.__next__()
        else:
            if not isinstance(p,SamplePoint):
                warnings.warn(f"pickled file contains an unexpected object of type: {type(p)}")
            return p
            
            
    def __exit__(self) -> None:
        if self._close:
            self.close()
            
    def read(self) -> typing.Optional[SamplePoint]:
        """
        Read through the iterator. Return None once the iterator's is reached
        """
        try:
            return self.__next__()
        except StopIteration:
            return None
    
    def close(self) -> None:
        self._file.close()
            
    
    
        
    
        