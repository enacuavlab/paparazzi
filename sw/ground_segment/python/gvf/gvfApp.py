#!/usr/bin/env python3

import wx
#from gvfframe import GVFFrame 
from modernGVFframe import GVFFrame
import argparse

class MessagesApp(wx.App):
    def __init__(self, wtf, ac_id, dims=2):
        self.ac_id = ac_id
        self.dims = dims
        wx.App.__init__(self, wtf)

    def OnInit(self):
        self.main = GVFFrame(self.ac_id,self.dims)

        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    parser = argparse.ArgumentParser("gvfApp", description="Python App for visualization GVF while PaparazziUAV is running")
    parser.add_argument('id_aircraft',type=int,help="ID of the aircraft one wants to plot its GVF")
    parser.add_argument('-d','--dim',dest='dim',choices=['2','3'],default=2,
                        help="Dimension asked for plotting (either 2 or 3, default is 2)")
    
    args = parser.parse_args()
    id_ac = args.id_aircraft
    dims = int(args.dim)
    application = MessagesApp(0, id_ac, dims)
    application.MainLoop()

if __name__ == '__main__':
    main()
