import math
import numpy as np
from rplidar import RPLidar, RPLidarException


class LiVFH:
    def __init__(self, robot_radius, safety_distance, turn_radius, measurement_threshold, wide_opening_deg=10):
        self.robot_radius = robot_radius
        self.safety_distance = safety_distance
        self.turn_radius = turn_radius
        self.threshold = measurement_threshold
        self.current_direction = 0
        self.former_direction = 0
        self.mu1 = 5
        self.mu2 = 2
        self.mu3 = 2
        self.robot_radius_safety_distance = self.robot_radius + self.safety_distance
        self.s_max = wide_opening_deg
        self.current_direction = 0
        self.former_direction = 0
        

    def get_lidar_data(self):
        """Reads raw data from the lidar 
        PARAMS:
            None
        RETURNS:
            list of float distances 
        """
        while True:
            lidar = RPLidar('/dev/tty.usbserial-0001')
            try:
                scan = next(lidar.iter_scans()) # just read one scan at a time
                # angles = [] 
                distances = []
                for (_, angle, distance) in scan:
                    # angles.append(angle)
                    distances.append(distance)
                print(f"Num measurements: {len(distances)}")
                print(distances)
                return distances
            except RPLidarException as e:
                # im sure there has to be a better way of handling errors
                # right now, so many errors are being caught 
                print(e)
                lidar.stop()
            finally:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
        # testing data
        #return [2396.25, 2409.25, 2461.25, 2472.0, 2536.0, 2552.25, 2628.0, 2643.25, 2672.75, 2703.0, 2768.75, 2806.25, 2868.25, 2914.0, 2986.0, 3041.5, 3120.25, 3173.5, 3257.25, 3325.25, 3430.0, 3505.25, 3626.25, 3704.5, 3712.25, 3830.25, 4113.75, 4204.75, 4334.0, 4396.25, 4572.25, 1954.75, 1841.0, 1858.25, 1861.5, 1867.25, 1873.25, 1888.75, 4427.75, 4407.0, 4374.25, 4350.5, 4337.5, 4301.25, 4287.5, 4276.25, 4276.5, 4289.25, 4295.75, 4223.5, 4218.0, 4225.25, 4319.5, 4394.25, 4724.75, 4554.75, 4566.5, 4569.75, 4565.25, 4564.75, 4592.0, 4600.5, 4578.75, 4803.5, 4808.25, 4820.5, 4353.75, 4346.0, 4367.75, 3329.0, 4401.25, 4428.0, 4467.25, 4484.0, 2579.5, 2621.25, 2731.25, 2833.75, 2873.5, 5208.75, 5277.5, 5315.75, 3564.75, 3572.25, 3529.0, 3221.0, 3199.5, 3229.25, 5645.5, 5831.0, 5582.75, 5259.75, 5407.25, 5488.5, 5497.75, 5298.25, 2790.5, 2790.0, 2813.5, 2848.5, 4911.5, 3394.0, 3393.0, 3344.25, 3302.0, 3232.75, 3176.0, 5882.5, 5821.0, 5746.0, 5817.0, 5815.0, 3925.75, 3906.0, 3887.5, 3883.5, 3913.25, 3926.75, 5500.25, 5557.5, 5599.5, 7666.25, 7761.0, 7999.5, 9212.25, 9212.0, 10498.75, 10411.5, 10350.5, 10425.75, 805.75, 4511.75, 865.5, 814.25, 820.75, 880.5, 859.75, 852.0, 833.75, 821.0, 824.0, 817.25, 840.0, 844.75, 841.75, 855.25, 865.5, 872.5, 885.5, 892.5, 895.75, 976.75, 982.5, 1006.0, 1027.75, 1452.75, 1433.5, 0, 933.25, 893.25, 881.75, 859.25, 858.75, 856.25, 856.0, 859.5, 872.0, 876.75, 609.0, 598.0, 586.75, 581.75, 577.75, 578.5, 578.25, 579.75, 587.25, 917.5, 926.75, 931.0, 536.5, 0, 0, 479.75, 474.0, 467.0, 458.0, 451.5, 449.5, 445.0, 438.25, 425.0, 424.0, 419.0, 415.0, 409.0, 407.0, 403.75, 400.0, 401.25, 401.0, 0, 0, 401.75, 400.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 928.0, 928.5, 0, 1074.75, 1080.75, 0, 0, 0, 1098.0, 1098.0, 1099.75, 1334.75, 2286.75, 2292.75, 2293.0, 2265.75, 2272.0, 2288.5, 2270.0, 2266.75, 0, 1645.25, 2272.75, 2273.25, 2279.25, 2279.75, 2293.5, 2294.0, 2302.5, 2305.75, 2314.75, 2321.0, 2336.0, 2346.0, 2372.0, 2391.5, 0, 0, 2400.5, 2388.0, 2425.25, 2416.75, 2412.0, 2450.75, 2466.25, 2417.25, 2512.25, 2537.5, 1362.25, 1370.0, 1389.5, 0, 0, 0, 1561.25, 3123.25, 3165.75, 0, 0, 2943.25, 2811.75, 2799.75, 2770.25, 2709.75, 2669.75, 2627.0, 2581.25, 2567.0, 2512.0, 2496.75, 2407.5, 2329.25, 2372.25, 2360.5, 2330.25, 2309.0, 2269.75, 2249.5, 2223.75, 2206.0, 2177.0, 2161.0, 2139.5, 2122.75, 2096.5, 2083.25, 2069.75, 2056.25, 2043.0, 2029.5, 2028.0, 2004.5, 2001.25, 1991.75, 1990.5, 1975.75, 1974.75, 1968.75, 1966.25, 1953.25, 1951.0, 1943.5, 1942.25, 1937.25, 1935.75, 1938.5, 1933.0, 1933.5, 1937.0, 1935.75, 1937.25, 1943.5, 1940.0, 1938.5, 1949.25, 1948.5, 1959.5, 1956.0, 1965.0, 1972.75, 1980.5, 1982.0, 1995.25, 1999.75, 2019.0, 2028.25, 2041.0, 2049.5, 2064.75, 2074.5, 2094.25, 2111.75, 2136.25, 2150.75, 2171.0, 2186.0, 2216.5, 2229.25, 2259.25, 2286.0, 2298.25, 2335.5, 2347.0]

    def read_laser_scan_data(self, file_path):
        """Reads data in from file and returns it as a list
        PARAMS:
            file_path (str): path to file
        RETURNS:
            list of floats read in from the file
        """
        with open(file_path, 'r') as file:
            laser_data = [float(value) for line in file for value in line.split()]
        print(laser_data)
        return laser_data
    
    def calculate_d2(self, lidar_scan: list) -> list:
        """Calculates the distance from turncircle to scanned object
        Parameters:
            Lidar_scan (list): Thresholded lidar_scan
        Returns:
            List of distances to scanned objects
        """
        distances = []
        for indx in range(len(lidar_scan)):
            if indx <=90:
                angle_to_obj = 90 - indx
            elif indx > 90 and indx <= 180:
                angle_to_obj = 180 - indx
            elif indx > 180 and indx <= 270:
                angle_to_obj = 270 - indx
            else:
                angle_to_obj = 360 - indx

            if lidar_scan[indx] != 0:
                distances.append(math.floor(math.sqrt(self.turn_radius**2 + lidar_scan[indx]**2 - 2 * self.turn_radius*lidar_scan[indx] * math.cos(angle_to_obj))))
            else:
                distances.append(np.inf)

        return distances

    def create_bin_histogram(self, lidar_scan: list) -> list:
        """Creates a binary histgram
        Parameters:
            lidar_scan (list): Thresholded lidar_scan
        Returns:
            Binary histogram (list)
        """
        bin_hist = []

        for scan in lidar_scan:
            if scan != 0:
                bin_hist.append(1)
            else:
                bin_hist.append(0)

        edge_hist = np.diff(bin_hist, 1) #Find edges in lidar scan
        edge_hist = list(edge_hist)
        edge_hist.insert(0, 0)

        
        for indx in range(len(bin_hist)):
            if edge_hist[indx] != 0:
                if edge_hist[indx] > 0:
                    angle_to_expand_by = self.enlarge_obstacle(lidar_scan[indx])
                    for index in range( indx - angle_to_expand_by, indx):
                            bin_hist[index] = 1
                else:
                    angle_to_expand_by = self.enlarge_obstacle(lidar_scan[indx - 1])
                    if (indx + angle_to_expand_by) >= len(bin_hist):
                        rest = indx + angle_to_expand_by - len(bin_hist)
                        for index in range(indx, len(bin_hist)):
                            bin_hist[index] = 1
                        for index in range(rest):
                            bin_hist[index] = 1
                    else:
                        for index in range(indx, indx + angle_to_expand_by):
                            bin_hist[index] = 1
        return bin_hist
    
    def enlarge_obstacle(self, distance) -> int:
        """Returns the amount the binary histogram needs to expand the scanned points by
        Parameters:
            distance (int): A single lidar_scan index, so a distance is given
        Returns:
            The distance in centimeters
        """
        angle_to_expand_by = round(((math.atan(self.robot_radius+self.safety_distance/distance)/2)/math.pi)*180)
        return angle_to_expand_by

    def calculate_is_blocked_right(self, lidar_scan: list) -> int:
        """Let's the system know if pathing to the right side of the robot is blocked
        Parameters:
            lidar_scan: list: Thresholded lidar_scan
        Returns:
            The right hand degree from where the histogram will be masked 
        """
        phi_r = 180
        distances = self.calculate_d2(lidar_scan=lidar_scan)
        for index in range(359, 180):
            if distances[index] < (self.turn_radius + (self.robot_radius + self.safety_distance)):
                phi_r = index
                return phi_r
        return phi_r

    def calculate_is_blocked_left(self, lidar_scan: list) -> int:
        """Let's the system know if pathing to the left side of the robot is blocked
        Parameters:
            lidar_scan: list: Thresholded lidar_scan
        Returns:
            The left hand degree from where the histogram will be masked 
        """
        phi_l = 180
        distances = self.calculate_d2(lidar_scan=lidar_scan)
        for index in range(0, 26):
            if distances[index] < (self.turn_radius + (self.robot_radius + self.safety_distance)):
                phi_l = index
                return phi_l
        return phi_l

    def create_masked_histogram(self, binary_histogram: list, lidar_scan: list) -> list:
        """Takes the binary histogram, and masks it, so only the reachable directions, are considered
        Parameters:
            binary_hisogram (list):  A binary histogram
            lidar_scan (list):       Thresholded lidar_scan
        Returns:
            A masked binary histogram
        """
        phi_l = self.calculate_is_blocked_left(lidar_scan)
        phi_r = self.calculate_is_blocked_right(lidar_scan)

        for index in range(len(binary_histogram)):
            if binary_histogram[index] == 0 and ((index >= 0 and index <= phi_l) or (index <= 359 and index >= phi_r)):
                pass
            else:
                binary_histogram[index] = 1
        return binary_histogram

    def find_candidates(self, masked_histogram: list, kt: int) -> list:
        """Finds the possible driving headings for the shortest distances around obstacles
        Parameters:
            masked_histogram (list): A masked histogram
            kt (int):                The direction of the goal
        Returns:
            A list of possible headings to drive (in degrees)
        """
        candidates_kr = []
        candidates_kl = []
        resulting_candidates = []
        wrap_around_kl = int
        is_wrapped_around = False

        padded_histogram = masked_histogram
        padded_histogram.insert(0, padded_histogram[-1])

        edge_histogram = np.diff(padded_histogram, 1)

        for index in range(len(edge_histogram)):
            if edge_histogram[index] == -1:
                candidates_kr.append(index)
            elif edge_histogram[index] == 1:
                if len(candidates_kr) == 0:
                    wrap_around_kl = index
                    is_wrapped_around = True
                else:
                    candidates_kl.append(index)

        if is_wrapped_around == True:
            candidates_kl.append(wrap_around_kl)
        
        for index in range(len(candidates_kl)):
            if (abs(candidates_kr[index] - candidates_kl[index])) <= self.s_max: # if opening is narrow
                resulting_candidates.append(int((candidates_kl[index] + candidates_kr[index])/2))
            else:
                cr = candidates_kr[index] + (self.s_max/2)
                if cr < 0:
                    cr = cr + 360
                cl = candidates_kl[index] - (self.s_max/2)
                if cl > 360:
                    cl = cl - 360
                resulting_candidates.append(int(cr))
                resulting_candidates.append(int(cl))
                if (kt >= cr and kt <= cl):
                    resulting_candidates.append(int(kt))
        if (is_wrapped_around == True and (kt >= resulting_candidates[-2] or kt <= resulting_candidates[-1])):
            resulting_candidates.append(int(kt))
            

        if len(resulting_candidates) == 0:
            resulting_candidates.append(kt)
        return resulting_candidates

    def cost_function(self, candidates: list, kt: int, current_direction: int, former_direction: int) -> list:
        """Grades the candidates, so that the lowest score is the best candidate. 
        Parameters:
            candidates (list):       A list of possible turning directions
            kt (int):                The directions towards the goal in degrees
            current_direction (int): The direction the robot is travling at the moment in degrees
            former_direction (int):  The direction the robot traveled last itteration in degrees
        Returns:
            A list of the scores, the individual candidates got, so that the indexes matches each other
        """
        scores = []
        delta_c_kt = int
        delta_c_current_direction = int
        delta_c_former_direction = int

        for index in range(len(candidates)):
            if abs(candidates[index] - kt) > 180:
                delta_c_kt = 360 - abs(candidates[index] - kt)
            else:
                delta_c_kt = abs(candidates[index] - kt)

            if abs(candidates[index] - current_direction) > 180:
                delta_c_current_direction = 360 - abs(candidates[index] - current_direction)
            else:
                delta_c_current_direction = abs(candidates[index] - current_direction)
            
            if abs(candidates[index] - former_direction) > 180:
                delta_c_former_direction = 360 - abs(candidates[index] - former_direction)
            else:
                delta_c_former_direction = abs(candidates[index] - former_direction)

            score = self.mu1 * delta_c_kt + self.mu2 * delta_c_current_direction + self.mu3 * delta_c_former_direction
            scores.append(score)
            
        return scores

    def find_steering_direction(self, lidar_scan: list, target_direction: int):
        """Finds the direction the robot needs to turn, to avoid obstacles
        Parameters:
            lidar_scan (list):       The raw lidar scan from the 360 degree lidar
            target_direction (int):  The direction the robot is traveling, in relationship to itself, in degrees
        Returns:
            The angle the robot needs to turn, in order to not hit an obstacle in radians
        """

        lidar_scan = [ scan%360 for scan in lidar_scan ]
        print(lidar_scan)
        #Remove any object the LiDAR detected that aren't within the threshold, and convert scan into centimeters
        thresholded_scan = list()
        for scan in lidar_scan:
            if scan == np.inf:
                thresholded_scan.append(2000)
            else:
                thresholded_scan.append(math.floor(scan * 100))
        
        for indx in range(len(thresholded_scan)):
            if thresholded_scan[indx] > self.threshold:
                thresholded_scan[indx] = 0

        #Create a positiv target direction, so that candidates can be picked out
        if target_direction < 0:
            target_direction_positive = target_direction + 360
        else:
            target_direction_positive = target_direction


        binary_histogram = self.create_bin_histogram(thresholded_scan) # Creates the binary histgram that needs to be masked
        masked_histogram = self.create_masked_histogram(binary_histogram=binary_histogram, lidar_scan=thresholded_scan) # Masks the binary histogram
        turning_candidates = self.find_candidates(masked_histogram=masked_histogram, kt=target_direction_positive) # Finds the candidates needed to make sure no collision with an obstacle is made

        #Makes sure that turning candidates that are over 180 degrees is negative, so the robot can turn right
        for index in range(len(turning_candidates)):
            if turning_candidates[index] > 180:
                turning_candidates[index] = turning_candidates[index] - 360

        # if TEST:
        #     binary_hist_msg = LaserScan()
        #     candidates_msg = LaserScan()
        #     hist_pub = rospy.Publisher("/obstacle/histogram", LaserScan, queue_size=1024)
        #     candidate_pub = rospy.Publisher("/obstacle/candidates", LaserScan, queue_size=1024)

        #     print(turning_candidates)
        #     candidates_hist = list(np.zeros(len(binary_histogram)))
        #     for candidate in turning_candidates:
        #         candidates_hist[candidate] = 1

        #     candidates_msg.ranges = candidates_hist
        #     binary_hist_msg.ranges = masked_histogram
            
        #     hist_pub.publish(binary_hist_msg)
        #     candidate_pub.publish(candidates_msg)

            

        #Scores the candidates, so the best candidate has the lowest score
        scores = self.cost_function(candidates=turning_candidates, kt=target_direction, current_direction=self.current_direction, former_direction=self.former_direction)

        #Finds the lowest score, and relates finds the index. That index denotes the the index that the best candidate has.
        best_score = min(scores)
        direction_to_turn = turning_candidates[scores.index(best_score)]

        #Sets current and former direction
        self.former_direction = self.current_direction
        self.current_direction = direction_to_turn

        #Makes direction_to_turn negative if it's over 180
        if direction_to_turn > 180:
            direction_to_turn -= 360

        return direction_to_turn * (math.pi/180) # Returns direction in radians

   

if __name__ == "__main__":
    # Create an instance of LiVFH
    robot_radius = 10  # Replace with your robot's radius in centimeters
    safety_distance = 20  # Replace with your desired safety distance in centimeters
    turn_radius = 30  # Replace with your desired turn radius in centimeters
    measurement_threshold = 1000  # Replace with your measurement threshold in centimeters

    obstacle_avoidance = LiVFH(robot_radius, safety_distance, turn_radius, measurement_threshold)

    # Read laser scan data from the file
    # laser_scan_data = obstacle_avoidance.read_laser_scan_data('laser_scan_data.txt')

    # Get laser scan data from get_lidar_data() method
    laser_scan_data = obstacle_avoidance.get_lidar_data()

    # Set a target direction (in degrees)
    target_direction_deg = 45  # Replace with your desired target direction in degrees

    # Find the direction to turn for obstacle avoidance (in radians)
    direction_to_turn_rad = obstacle_avoidance.find_steering_direction(laser_scan_data, target_direction_deg)

    print("Direction to Turn:", math.degrees(direction_to_turn_rad), "degrees")
