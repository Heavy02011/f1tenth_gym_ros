"""
Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.

"""
import math
RADDEG = 180. / math.pi 
DEGRAD = math.pi / 180.

class LidarScan(object):
    def __init__(self, scan_msg):

        # save Lidar Scan message
        self.scan_msg = scan_msg
        #print(scan_msg)

        # timestamp in the header is the acquisition time of 
        # the first ray in the scan:
        # in frame frame_id, angles are measured around 
        # the positive Z axis (counterclockwise, if Z is up)
        # with zero angle being forward along the x axis
        self.header = scan_msg.header
        #print(self.header)

        # start angle of the scan [rad]
        self.angle_min = scan_msg.angle_min
        
        # end angle of the scan [rad]
        self.angle_max = scan_msg.angle_max
        #print(self.angle_min*RADDEG, self.angle_max*RADDEG) -135/+135

        # angular distance between measurements [rad]
        self.angle_increment = scan_msg.angle_increment
        #print(self.angle_increment)

        # time between measurements [seconds] - if your scanner
        # is moving, this will be used in interpolating position
        # of 3d points
        self.time_increment = scan_msg.time_increment
        #print(self.time_increment)
                         
        # time between scans [seconds]
        self.scan_time = scan_msg.scan_time        
        #print(self.scan_time)

        # minimum range value [m]
        self.range_min =  scan_msg.range_min        

        # maximum range value [m]
        self.range_max = scan_msg.range_max
        #print(self.range_min, self.range_max)

        # range data [m] (Note: values < range_min or > range_max should be discarded)
        self.ranges = scan_msg.ranges
        #print(self.ranges)

        # intensity data [device-specific units].  If your
        # device does not provide intensities, please leave
        # the array empty.
        self.intensities = scan_msg.intensities
           
    def get_ranges(self):

        # loop through ranges
        angle = self.angle_min

        # distances of obstacles or lines to left, right, front
        dist_left  = 9999999999
        dist_front = 9999999999
        dist_right = 9999999999
        idx_front  = 0
        idx_left   = 0
        idx_right  = 0

        idx = 0
        # angle segment relevant to detect minimum distance to left & right side of vehicle
        del_side = 5.0 * DEGRAD

        # angle segment relevant to detect minimum distance to the front of vehicle
        del_front = 30.0 * DEGRAD


        for range in self.ranges:
            angle += self.angle_increment
            #print(idx, angle * RADDEG, range)
            
            # cut ranges lower min or greater than max
            if (range >= self.range_min) and (range <= self.range_max):
                
                # check right side
                if (angle > -90*DEGRAD-del_side) and (angle < -90*DEGRAD+del_side):
                    if range < dist_right:
                        dist_right = range
                        idx_right = idx

                # check FRONT
                if (angle > -del_front) and (angle < del_front):
                    if range < dist_front:
                        dist_front = range
                        idx_front = idx

                # check left side
                if (angle > 90*DEGRAD-del_side) and (angle < 90*DEGRAD+del_side):
                    if range < dist_left:
                        dist_left = range
                        idx_left = idx

            # count lidar scan points
            idx += 1

        return dist_left, dist_front, dist_right

        # cover left & right lane
        # cover front
        # cover back