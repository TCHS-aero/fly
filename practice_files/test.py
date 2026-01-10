import csv

file = "mission_waypoints.csv"


def get_waypoints(file):
    waypoints = []

    with open(file, "r") as f:
        reader = csv.reader(f)
        next(reader)

        for row in reader:
            waypoints.append(
                [
                    float(row[0]),
                    float(row[1]),
                    float(row[2]),
                    int(row[3]),
                ]
            )

    return waypoints


def get_distance_from_waypoint_as_percentage(waypoint: list):
    # distance formula or manhattan formula to grab the distance
    # Check that distance over 100
    # multiply times 100
    # return percetange
    pass


async def main():
    waypoints = get_waypoints(file)
    number_of_waypoints = len(waypoints)

    for waypoint in waypoints:
        # unpack the data
        # Fly from home to the first waypoint
        # Wait until we get there
        # Continue with the next waypoint
        # Track progress by using the number_of_waypoints
        pass


list1 = [1, 2, 3]
list2 = [4, 5, 6]

list1.extend([list2])
print("list1", list1)
print("list2", list2)
