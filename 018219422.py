"""
Script Description:
-------------------

This Python script implements Dijkstra's & A* algorithm to find the shortest path between two nodes in a graph. The graph is defined by the input files `input.txt` and `coords.txt`, which should be located in the same directory as this script.

Key Features:
- **Graph Input Processing**: Reads graph edges and nodes from `input.txt` and node coordinates from `coords.txt`.
- **Shortest Path Calculation**: Uses a priority queue to efficiently compute the shortest path between the specified start and end nodes.
- **Visualization**: Generates images at each step of the algorithm to visualize the search process.
- **Video Generation**: Optionally creates a video from the generated images using FFmpeg.
- **Command-Line Interface**: Allows control over video generation via command-line arguments.
- **Output**: Writes the shortest path and corresponding distances to a text file named `<SJSU_ID>.txt`.


Usage:
------

Run the script with the option to generate a video (`--video 1`) or not (`--video 0`):
Run the script with the option to determine video frame rate by (`--steps_per_frame FRAME_RATE`):

```bash
python main.py --video 1  --steps_per_frame 3  # Generates the video
python main.py --video 0  # Does not generate the video
```
"""
from collections import defaultdict
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from io import BytesIO
import numpy as np
import cv2
import argparse
from tqdm import tqdm
import logging
import time
import psutil

SJSU_ID = "018219422"
logging.basicConfig(level=logging.INFO)


class Algorithm:
    def __init__(self, name, graph_info, node_info, start_node, end_node):
        self.name = name
        self.graph_info = graph_info
        self.node_info = node_info
        self.start_node = start_node
        self.end_node = end_node
        self.open_set = []
        self.closed_set = set()
        self.min_distance_from_start = defaultdict(lambda: float('inf'))
        self.min_distance_from_start[start_node] = 0.0
        self.path = []
        self.distance_history = []
        self.visualize_dict = {
            "node_searched": [],  # black
            "node_searching": [],  # blue
        }
        self.result = ([], [])
        self.step = 0
        self.previous_distance = 0
        self.finished = False
        # Metrics
        self.iterations = 0
        self.execution_time = 0.0
        self.memory_usage = 0
        self.process = psutil.Process()  # Process for memory tracking
        self.max_memory = 0

    def heuristic(self, current_node):
        # Override in subclasses if needed
        return 0

    def step_algorithm(self):
        # Override in subclasses
        pass

    def update_memory_usage(self):
        current_memory = self.process.memory_info().rss
        self.max_memory = max(self.max_memory, current_memory)


class DijkstraAlgorithm(Algorithm):
    def __init__(self, graph_info, node_info, start_node, end_node):
        super().__init__('Dijkstra', graph_info, node_info, start_node, end_node)
        heapq.heappush(self.open_set, (0, [self.start_node], [0]))
        self.start_time = time.time()

    def step_algorithm(self):
        if not self.open_set or self.finished:
            if not self.finished:
                self.execution_time = time.time() - self.start_time
                self.finished = True
            return

        self.iterations += 1  # Increment iterations
        self.update_memory_usage()  # Update memory usage

        current_distance, path, distance_history = heapq.heappop(self.open_set)
        current_node = path[-1]

        if current_node in self.closed_set:
            return

        self.closed_set.add(current_node)
        self.min_distance_from_start[current_node] = current_distance

        # Visualization
        self.visualize_dict["node_searching"].append(self.node_info[current_node])

        if current_node == self.end_node and not self.result[0]:
            self.result = (path, distance_history)
            self.execution_time = time.time() - self.start_time
            self.finished = True

        for next_node, diff_distance in self.graph_info[current_node]:
            if next_node in self.closed_set:
                continue

            next_distance = current_distance + diff_distance

            if next_distance < self.min_distance_from_start[next_node]:
                self.min_distance_from_start[next_node] = next_distance
                heapq.heappush(self.open_set, (next_distance, path + [next_node], distance_history + [next_distance]))


class AStarAlgorithm(Algorithm):
    def __init__(self, epsilon, graph_info, node_info, start_node, end_node):
        super().__init__(f'A*_epsilon_{epsilon}', graph_info, node_info, start_node, end_node)
        self.epsilon = epsilon
        heapq.heappush(self.open_set, (0 + self.epsilon * self.heuristic(self.start_node), 0, [self.start_node], [0]))
        self.start_time = time.time()

    def heuristic(self, current_node):
        # Euclidean distance between current node and end node
        x1, y1 = self.node_info[current_node]
        x2, y2 = self.node_info[self.end_node]
        return np.hypot(x2 - x1, y2 - y1)

    def step_algorithm(self):
        if not self.open_set or self.finished:
            if not self.finished:
                self.execution_time = time.time() - self.start_time
                self.finished = True
            return

        self.iterations += 1  # Increment iterations
        self.update_memory_usage()  # Update memory usage

        est_total_cost, current_distance, path, distance_history = heapq.heappop(self.open_set)
        current_node = path[-1]

        if current_node in self.closed_set:
            return

        self.closed_set.add(current_node)
        self.min_distance_from_start[current_node] = current_distance

        # Visualization
        self.visualize_dict["node_searching"].append(self.node_info[current_node])

        if current_node == self.end_node and not self.result[0]:
            self.result = (path, distance_history)
            self.execution_time = time.time() - self.start_time
            self.finished = True

        for next_node, diff_distance in self.graph_info[current_node]:
            if next_node in self.closed_set:
                continue

            tentative_g_score = current_distance + diff_distance

            if tentative_g_score < self.min_distance_from_start[next_node]:
                self.min_distance_from_start[next_node] = tentative_g_score
                est_total_cost = tentative_g_score + self.epsilon * self.heuristic(next_node)
                heapq.heappush(self.open_set, (est_total_cost, tentative_g_score, path + [next_node], distance_history + [tentative_g_score]))


class DataProcessor:
    def __init__(self, make_video: bool, steps_per_frame: int):
        self.m = 0
        self.n = 0
        self.start_node = 0
        self.end_node = 0
        self.graph_info = defaultdict(list)
        self.node_info = [(None, None)]
        self.make_video = make_video
        self.steps_per_frame = int(steps_per_frame)
        self.algorithms = []
        self.frames = []

    def process_input_files(self):
        logging.info("[Data Preprocessing] :: input.txt")
        input_info = []
        with open('input.txt', 'r') as f:
            for context in f:
                input_info.append(context.rstrip('\n').split(' '))

        _, self.start_node, self.end_node = input_info[:3]
        self.start_node = int(self.start_node[0])
        self.end_node = int(self.end_node[0])

        distance_info = input_info[3:]
        distance_info.sort()
        for start, end, distance in distance_info:
            start, end, distance = int(start), int(end), float(distance)
            self.graph_info[start].append((end, distance))
            self.graph_info[end].append((start, distance))

        logging.info("[Data Preprocessing] :: coords.txt")
        with open('coords.txt', 'r') as f:
            for context in f:
                x, y = context.rstrip('\n').split(' ')
                x, y = float(x), float(y)
                self.m = max(self.m, x)
                self.n = max(self.n, y)
                self.node_info.append((x, y))

    def initialize_algorithms(self):
        logging.info("[Initializing Algorithms]")
        # Initialize Dijkstra's algorithm
        self.algorithms.append(DijkstraAlgorithm(self.graph_info, self.node_info, self.start_node, self.end_node))
        # Initialize A* algorithms with epsilons from 1 to 5
        for epsilon in range(1, 6):
            self.algorithms.append(AStarAlgorithm(epsilon, self.graph_info, self.node_info, self.start_node, self.end_node))

    def run_algorithms(self):
        logging.info("[Running Algorithms] :: Running all algorithms and generating frames")
        step_counter = 0
        max_steps = len(self.graph_info) * len(self.graph_info)
        with tqdm(total=max_steps, desc="Processing Steps") as pbar:
            while not all(algo.finished for algo in self.algorithms):
                for algo in self.algorithms:
                    if not algo.finished:
                        algo.step_algorithm()
                if step_counter % self.steps_per_frame == 0 and self.make_video:
                    self.generate_combined_image_for_step()
                step_counter += 1
                pbar.update(1)
        # Ensure that any remaining frames are generated
        if self.make_video:
            self.generate_combined_image_for_step()

    def generate_base_graph_image(self, algo):
        fig, ax = plt.subplots(figsize=(5, 5))
        # Draw all edges
        for node1, neighbors_info in self.graph_info.items():
            node1_x, node1_y = self.node_info[node1]
            for node2, _ in neighbors_info:
                node2_x, node2_y = self.node_info[node2]
                ax.plot([node1_x, node2_x], [node1_y, node2_y], 'b-', linewidth=0.5, alpha=0.2)

        # Draw all nodes
        for idx, (x, y) in enumerate(self.node_info[1:], start=1):
            ax.scatter(x, y, s=50, color='tab:blue', alpha=0.5)
            ax.text(x, y, str(idx), fontsize=9)

        # Start and end nodes
        start_x, start_y = self.node_info[self.start_node]
        end_x, end_y = self.node_info[self.end_node]
        ax.scatter(start_x, start_y, s=100, color='g', alpha=1)
        ax.scatter(end_x, end_y, s=100, color='r', alpha=1)

        return fig, ax

    def generate_combined_image_for_step(self):
        images = []
        for algo in self.algorithms:
            fig, ax = self.generate_base_graph_image(algo)
            # Draw searched nodes
            for x, y in algo.visualize_dict["node_searched"]:
                ax.scatter(x, y, color='tab:gray', s=100, alpha=1)
            # Draw searching nodes
            for x, y in algo.visualize_dict["node_searching"]:
                ax.scatter(x, y, color='b', s=100, alpha=1)
            # Clear searching nodes after plotting
            algo.visualize_dict["node_searched"].extend(algo.visualize_dict["node_searching"])
            algo.visualize_dict["node_searching"] = []
            ax.set_title(algo.name)
            buf = BytesIO()
            fig.savefig(buf, format='png')
            buf.seek(0)
            image = np.frombuffer(buf.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            images.append(image)
            plt.close(fig)

        # Combine images into a 2x3 grid
        row1 = np.hstack(images[:3])
        row2 = np.hstack(images[3:])
        combined_image = np.vstack((row1, row2))
        self.frames.append(combined_image)

    def generate_final_images(self):
        logging.info("[Data Visualization] :: Generating final images")
        for algo in self.algorithms:
            fig, ax = self.generate_base_graph_image(algo)
            path = algo.result[0]
            for i in range(len(path) - 1):
                node1 = path[i]
                node2 = path[i + 1]
                node1_x, node1_y = self.node_info[node1]
                node2_x, node2_y = self.node_info[node2]
                ax.plot([node1_x, node2_x], [node1_y, node2_y], 'r-', linewidth=3, alpha=1)
            ax.set_title(algo.name + " - Final Path")
            buf = BytesIO()
            fig.savefig(buf, format='png')
            buf.seek(0)
            image = np.frombuffer(buf.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            self.frames.append(image)
            plt.close(fig)

    def generate_output_file(self):
        logging.info("[Data Result] :: Generating output files")
        # Write paths and distances to <SJSU_ID>.txt
        with open(f"{SJSU_ID}.txt", "w") as file:
            for algo in self.algorithms:
                path = algo.result[0]
                distance_history = algo.result[1]
                node_info_string = " ".join([str(x) for x in path])
                distance_info_string = " ".join([f"{x:.5f}" for x in distance_history])
                file.write(node_info_string + '\n' + distance_info_string + '\n')
        # Write metrics to algorithm_cost.txt
        with open("algorithms_cost.txt", "w") as file:
            for algo in self.algorithms:
                memory_in_kb = algo.max_memory / 1024  # Convert bytes to kilobytes
                file.write(f"{algo.name}\n")
                file.write(f"Total Iterations: {algo.iterations}\n")
                file.write(f"Execution Time: {algo.execution_time:.6f} seconds\n")
                file.write(f"Peak Memory Usage: {memory_in_kb:.2f} KB\n\n")

    def generate_video_from_frames(self):
        logging.info(f"[Data Visualization] :: Generating video, steps_per_frame = {self.steps_per_frame}")
        height, width, layers = self.frames[0].shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter(f"{SJSU_ID}.mp4", fourcc, 10, (width, height))
        for frame in self.frames:
            video.write(frame)
        video.release()

    def main(self):
        start_time = time.time()
        self.process_input_files()
        self.initialize_algorithms()
        self.run_algorithms()
        self.generate_final_images()
        self.generate_output_file()
        if self.make_video:
            self.generate_video_from_frames()
        end_time = time.time()
        logging.info(f"Visualizing Algorithms completed. Total time = {end_time - start_time:.3f}[s]")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Algorithms (#018219422)")
    parser.add_argument('--video', type=int, default=1, help="Set to 1 to generate video, 0 otherwise.")
    parser.add_argument('--steps_per_frame', type=int, default=3, help="Increase value to include more steps in one frame. Default = 3")
    args = parser.parse_args()

    dp = DataProcessor(make_video=bool(args.video), steps_per_frame=args.steps_per_frame)
    dp.main()
