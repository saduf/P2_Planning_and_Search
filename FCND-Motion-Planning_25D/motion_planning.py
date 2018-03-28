import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import networkx as nx
from sampling import Sampler

from planning_utils import a_star_graph, heuristic_graph, prune_path, closest_point
from planning_utils import can_connect, create_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):  
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 3.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            #if self.global_position[2] - self.global_home[2] < 0.1:
            if abs(self.local_position[2]) < 0.01 or np.linalg.norm(self.local_velocity[2]) < .01:
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 7

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)  # gets the first line
            lat0, lon0 = float(row1[0][5:]), float(row1[1][5:])

        # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)  # set the current location to be the home position

        # TODO: retrieve current global position
        current_global_pos  = (self._longitude, self._latitude, self._altitude)
 
        # TODO: convert to current local position using global_to_local()
        current_local_pos = global_to_local(current_global_pos, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

         # Get the center of the grid
        north_offset = int(np.floor(np.min(data[:, 0] - data[:, 3])))
        east_offset = int(np.floor(np.min(data[:, 1] - data[:, 4])))
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        sampler = Sampler(data)
        polygons = sampler._polygons

        # Example: sampling 1200 points and removing
        # ones conflicting with obstacles.
        nodes = sampler.sample(1200)
        print("Non collider nodes:", len(nodes))

        t0 = time.time()
        # Uncomment line 162 to generate the graph from the sampled points, and comment out line 163.
        # Increase the Mavlink timer to avoid disconnecting from the simulator.
        #G = create_graph(nodes, 10, polygons)
        G = nx.read_gpickle("graph_1200_SD_nodes.gpickle") # Comment out this line if generating the graph instead of using the saved graph 
        print('graph took {0} seconds to build'.format(time.time()-t0))    
        print("Number of edges", len(G.edges))

        # TODO: convert start position to current position rather than map center
        grid_start = (int(current_local_pos[0]-north_offset), int(current_local_pos[1]-east_offset), TARGET_ALTITUDE)
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_global_pos = (-122.394700, 37.789825, 13)
        goal_local_pos = global_to_local(goal_global_pos, self.global_home)
        grid_goal = (int(goal_local_pos[0]-north_offset), int(goal_local_pos[1]-east_offset), goal_global_pos[2])
        print ("goal_local_N:", goal_local_pos[0], "goal_local_E:", goal_local_pos[1], "goal_local_alt:", goal_local_pos[2])

        #goal_ne = (455., 635., 20.)
        start_ne_g = closest_point(G, grid_start)
        goal_ne_g = closest_point(G, grid_goal)
        print('Local Start and Goal: ', start_ne_g, goal_ne_g)

        # Run A* to find a path from start to goal
        path, cost = a_star_graph(G, heuristic_graph, start_ne_g, goal_ne_g)
        print("Path Length:", len(path), "Path Cost:", cost)

        int_path = [[int(p[0]), int(p[1]), int(p[2])] for p in path]        
    
        # TODO: prune path to minimize number of waypoints
        pruned_path = prune_path(int_path)
        print("Length Pruned Path:", len(pruned_path))
        print ("PRUNED PATH:", pruned_path)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, p[2], 0] for p in pruned_path]

        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)
    
    drone.start()

