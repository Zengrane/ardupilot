#!/usr/bin/env python
'''
Single Marker Module
This module adds a single marker at a predefined position on the MAVMap.
'''

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class SingleMarkerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialize the module"""
        super(SingleMarkerModule, self).__init__(mpstate, "single_marker", "")
        
        # Predefined position (latitude, longitude)
        self.marker_lat = -35.36341649  # Replace with your desired latitude
        self.marker_lon = 149.16525123  # Replace with your desired longitude

        # Add a command to place the marker
        self.add_command('add_marker', self.cmd_add_marker, "Add a single marker on the map")

    def cmd_add_marker(self, args):
        '''Add a marker at the predefined location'''
        # Add a marker on the map at the predefined position
        self.mpstate.map.add_marker(self.marker_lat, self.marker_lon, "Single Marker")
        print(f"Marker added at ({self.marker_lat}, {self.marker_lon})")

def init(mpstate):
    '''Initialize the module'''
    return SingleMarkerModule(mpstate)
