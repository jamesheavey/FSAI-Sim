import math
from .analyse_map import AnalyseMap


class FSAIRules:

    @staticmethod
    def remove_intersections(points):
        """
        Function to find and remove intersection.
        It does this by:
            Locating the points that form the loop that creates the intersection
            Reversing the order of the points that form the loop
        """
        loop_points = []
        points_index = []
        test_points = []
        intersections, positions, spline_points = AnalyseMap.determine_intersections(points)
        number_of_intersections = len(intersections)
        loop_count = 0
        if number_of_intersections < 3:
            # Find corresponding indices of points for each intersection
            for i in range(len(intersections)):
                if positions[i][0] > positions[i][1]:
                    for index in range(positions[i][0], positions[i][1]):
                        for k in range(len(points)):
                            if math.dist(spline_points[index], points[k]) > 4:
                                continue
                            loop_points.append(points[k])
                            points_index.append(k)

                # Remove duplicate values from both lists
                loop_points = list(dict.fromkeys(loop_points))
                points_index = list(dict.fromkeys(points_index))

                if 0 < len(loop_points) < 3:
                    for index in points_index:
                        points.remove(index)
                    continue
                # Reverse the order of the items within the loop
                loop_points.reverse()
                z = 0
                for index in points_index:
                    test_points[index] = loop_points[z]
                    z += 1
                intersections, positions, spline_points = AnalyseMap.determine_intersections(points)
                # If number of intersections is reduced, update points
                if len(intersections) < number_of_intersections:
                    points = test_points
                    continue
                loop_count += 1

                number_of_intersections = len(intersections)
        return points, intersections

    @staticmethod
    def remove_tight_corners(points, track_width):
        """
        returns a modified list of points after checking for corners that are too tight
        """
        radii, x_new, y_new, u_new = AnalyseMap.calculate_track_radii(points)

        tight_corners = AnalyseMap.find_tight_corners(radii, x_new, y_new, track_width, 5)
        tight_corner_points = AnalyseMap.tight_corner_points(tight_corners, points)
        if len(points) - len(tight_corner_points) > 3:
            points = [v for i, v in enumerate(points) if i not in tight_corner_points]
        return points

    @staticmethod
    def remove_overlaps(points, track_width):
        """
        Removes overlapping points
        """
        overlap_points = []
        overlaps = AnalyseMap.find_overlaps(points, track_width)
        print(f"points = {points}")
        for i in range(len(overlaps)):
            if overlaps[i][0] > overlaps[i][1]:
                overlap_points.append(points[overlaps[i][0]])
                overlap_points.append(points[overlaps[i][1]])
                print(f"overlap points = {overlap_points}")
                average = [sum(x)/len(x) for x in zip(*overlap_points)]
                print(f"average = {average}")

                for i, point in enumerate(points):
                    if math.dist(average, point) < 10:
                        vector = (point[0] - average[0], point[1] - average[1])
                        new_point = (average[0] + (1.5 * vector[0]), average[1] + (1.5 * vector[1]))
                        points[i] = new_point

                print(f"points = {points}")
        return points

    @staticmethod
    def adhere_to_rules(points, track_width, allow_intersections):
        """
        Returns if the generated track adheres to the FS-AI rules based on track radii and intersections
        """
        intersections, _, _ = AnalyseMap.determine_intersections(points)
        radii, x_new, y_new, _ = AnalyseMap.calculate_track_radii(points)
        tight_corners = AnalyseMap.find_tight_corners(radii, x_new, y_new, track_width, 4.5)

        if not allow_intersections and len(intersections) == 0 and len(tight_corners) == 0:
            return True
        elif allow_intersections and len(tight_corners) == 0:
            return True
        return False
