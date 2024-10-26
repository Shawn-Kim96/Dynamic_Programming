"""
Script Description:
-------------------

This Python script implements Dynamic Programming to find the shortest path between two nodes in a graph. The graph is defined by the input files `input.txt` and `coords.txt`, which should be located in the same directory as this script.

Key Features:
- **Graph Input Processing**: Reads graph edges and nodes from `input.txt` and node coordinates from `coords.txt`.
- **Shortest Path Calculation**: Uses a deque to compute the shortest path between the specified start and end nodes using cost-to-come.
- **Output**: Writes the shortest path and optimal solution for all nodes to a text file named `<SJSU_ID>.txt`.


Usage:
------

Run the script with python 018219422.py
```bash
python 018219422.py
```
"""

from collections import defaultdict, deque
from tqdm import tqdm
import logging
import time

SJSU_ID = "018219422"
logging.basicConfig(level=logging.INFO)


class DataProcessor:
    def __init__(self):
        self.m = 0
        self.n = 0
        self.start_node = 0
        self.end_node = 0
        self.node_count = 0
        self.edges_child_info = defaultdict(list)  # graph with data structure of hash map; start_node: [(node1, distance1), (node2, distance2), ... ]
        self.node_info = [(None, None)]  # node_info[i] = ith node (x, y), 0th index have no meaning
    
    def process_input_files(self):
        """
        preprocess input files.
        """
        # process input.txt
        logging.info("[Data Preprocessing] :: input.txt")
        input_info = []
        with open('input.txt', 'r') as f:
            for context in f:
                input_info.append(context.rstrip('\n').split(' '))

        # 1. get the information from first 3 lines
        self.node_count, self.start_node, self.end_node = input_info[:3]
        self.node_count = int(self.node_count[0])
        self.start_node = int(self.start_node[0])
        self.end_node = int(self.end_node[0])

        # 2. get the information about distance from 4th line
        distance_info = input_info[3:]
        distance_info.sort()  # to maintain node order (starting from small)
        for start, end, distance in distance_info:
            start, end, distance = int(start), int(end), float(distance)
            self.edges_child_info[start].append((end, distance))

        # process coords.txt
        logging.info("[Data Preprocessing] :: coords.txt")
        with open('coords.txt', 'r') as f:
            for context in f:
                x, y = context.rstrip('\n').split(' ')
                x, y = float(x), float(y)
                self.m = max(self.m, x)
                self.n = max(self.n, y)
                self.node_info.append((x, y))
        

    def dynamic_programming(self):
        """
        method: use list of distance(start - node) as a memoization

        1. initialize memoization list to float('inf')
        2. for nodes with zero ranking, caclulate does with dynamic programming
        """

        queue = deque([self.start_node])
        dp = [[float('inf'), []] for _ in range(self.node_count + 1)]  # dp[i] = [distance between start - node[i], path]
        dp[self.start_node] = [0, [self.start_node]]

        with tqdm(desc="Processing Nodes") as pbar:
            while queue:
                node = queue.popleft()

                for next_node, distance in self.edges_child_info[node]:
                    cost_to_come, path_to_come = dp[node]
                    if cost_to_come + distance < dp[next_node][0]:
                        dp[next_node] = [cost_to_come + distance, path_to_come + [next_node]]
                        queue.append(next_node)
                
                pbar.update(1)

        return dp
        
    def generate_output_file(self, result):
        """
        generate output.txt file.
        1. optimal path
        2. optimal value functions for all nodes
        """
        optimal_path = result[self.end_node][1]
        optimal_value_function = [x[0] for x in result[1:]]
        
        op_string = ", ".join(map(str, optimal_path))
        ovf_string = ", ".join(map(str, optimal_value_function))

        with open(f"{SJSU_ID}.txt", "w") as file:
            file.write(op_string + '\n' + ovf_string)

    def main(self):
        start_time = time.time()
        self.process_input_files()
        dp_result = self.dynamic_programming()
        self.generate_output_file(dp_result)
        end_time = time.time()
        logging.info(f"Dynamic Programming calculation completed. Total time = {end_time - start_time:.3f}[s]")


if __name__ == "__main__":
    logging.info("Run Dynamic programming (#018219422)")
    dp = DataProcessor()
    dp.main()
