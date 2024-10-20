#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap


class MapLoader(Node):
    def __init__(self):
        super().__init__("map_loader_node")

        self.maps_folder = os.path.expanduser('~/ros2_tk/src/task_bot/maps')


        self.run()


    # running the loop continuously for loading the different maps
    def run(self):
        while rclpy.ok():
            self.maps = [f for f in os.listdir(self.maps_folder) if f.endswith('.yaml')]

            if not self.maps:
                self.get_logger().info("No maps found, Please check the directory")
                return
            
            self.print_map_options()

            selected_map = self.get_user_selection()

            if selected_map is not None:
                self.call_map_service(selected_map)

            else:
                print("Exiting.. ")
                break


    def print_map_options(self):
        print("Available Maps: ")
        for idx, map_file in enumerate(self.maps,1):
            print(f"{idx}.{map_file}")
        print(f"0. Exit")


    # getting the user input
    def get_user_selection(self):
        try:
            user_input = int(input("Enter the Map number to load"))
            if 1<= user_input <=len(self.maps):
                return self.maps[user_input - 1]
            else:
                print("Invalid Selection ")
                return None
        except ValueError:
            print("Please enter a valid number")
            return None
        
    # by using the service call method we change the map based on user input
    def call_map_service(self,map_file):
        self.client = self.create_client(LoadMap,'/map_server/load_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting... ")

        req = LoadMap.Request()
        req.map_url = os.path.join(self.maps_folder,map_file)

        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info(f"Map '{map_file}' loaded succesfully")
        else:
            self.get_logger().error("Failed to load the map")

def main(args=None):
    rclpy.init(args=args)
    try:
        map_loader = MapLoader()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()