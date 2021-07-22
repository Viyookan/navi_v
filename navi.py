


    def is_region_intersecting(ax, ay, az, ar, bx, by, bz, br):
        """
        This function takes in 2 series of circles and checks if any of these circle-pairs intersect using vector
        computation of the following formula: 2 circles intersect by checking if the distance between the centres of the
        2 circles (D = sqrt((x1 - x2)^2 + (y1 - y2)^2 +(z1 - z2)^2)) is greater than the sum of their radii
        (R = r1 + r2), if D >= R then the circle-pair is not intersecting, otherwise they are either concentric or
        intersecting.

        The function returns true if the formula returns true for any of these circle-pairs.

        return np.any((ar + br) ** 2 >= np.power((ax - bx), 2) + np.power((ay - by), 2) + np.power((az - bz), 2))

    def is_trajectory_intersecting(at, bt, start_time, end_time):
        """
        This function takes in 2 trajectories (t, x, y, z), where t, x, y, z are numpy float arrays, extracts subsets
        with equal length from the 2 trajectories between start_time <= t <= end_time and tests if these subsets are
        intersecting with the is_region_intersecting function.

        :param at: numpy float array
        :param bt: numpy float array
        :param start_time: float
        :param end_time: float
        :return: bool
        """

        # Extracts subsets with equal length
        a_subset = at[(at[:, 0] >= start_time) & (at[:, 0] <= end_time)]
        b_subset = bt[(bt[:, 0] >= start_time) & (bt[:, 0] <= end_time)]

        # Check for any intersections
        return is_region_intersecting(ax=a_subset[:, 1], ay=a_subset[:, 2], az=a_subset[:, 3], ar=a_subset[:, 4],
                                      bx=b_subset[:, 1], by=b_subset[:, 2], bz=b_subset[:, 3], br=b_subset[:, 4])


    def generate_trajectory(start_region, end_region, time_interval, lateral_speed, vertical_speed):
        """
        This function takes in a start_point (t1, x1, y1, z1, s1) and end_point comprising (t2, x2, y2, z2, s2), where
        t, x, y, z, s are python floats and integers, and generates an array of numpy float values that represents the
        position and size of an AGV between time t2 - t1.

        Notes:
            1. Movement along the horizontal and vertical planes are assumed to be independent, hence the motion has to
            be resolved upstream.

        :param start_region: region tuple (t, x, y, z, s)
        :param end_region: region tuple (t, x, y, z, s)
        :param time_interval: int
        :param lateral_speed: float
        :param vertical_speed: float
        :return: numpy list of trajectory tuples [(t, x, y, z, s), ...]
        """

        def get_direction(end, start):
            return -1 if end - start < 0 else 1

        # Extract size
        size = start_region[4]

        # Resolve time
        start_time, end_time = start_region[0], end_region[0]
        window = end_time - start_time

        # Resolve horizontal motion components
        x, y = start_region[1], start_region[2]
        x_direction, y_direction = get_direction(end_region[1], start_region[1]), get_direction(end_region[2], start_region[2])
        x_distance, y_distance = abs(end_region[1] - start_region[1]), abs(end_region[2] - start_region[2])

        if x_distance or y_distance:  # This is the case where the AGV is moving
            theta = np.arctan(y_distance / x_distance) if x_distance != 0 else np.pi / 2  # Arctangent is infinite at pi/2 and -pi/2
            x_velocity, y_velocity = lateral_speed * np.cos(theta), lateral_speed * np.sin(theta)
        else:  # This is the stationary case
            x_velocity, y_velocity = 0.0, 0.0

        # Resolve vertical motion components
        z = start_region[3]
        z_direction = get_direction(end_region[3], start_region[3])
        z_distance = abs(end_region[3] - start_region[3])
        z_velocity = vertical_speed if z_distance > 0.0 else 0.0

        # Generate trajectories
        return np.column_stack((np.arange(start_time, end_time, time_interval, dtype=int).reshape(-1, 1),
                                (x + x_direction * x_velocity * time_interval * np.arange(0, window, time_interval) / time_interval).reshape(-1, 1),
                                (y + y_direction * y_velocity * time_interval * np.arange(0, window, time_interval) / time_interval).reshape(-1, 1),
                                (z + z_direction * z_velocity * time_interval * np.arange(0, window, time_interval) / time_interval).reshape(-1, 1),
                                np.full((window / time_interval, ), size).reshape(-1, 1)))


 
