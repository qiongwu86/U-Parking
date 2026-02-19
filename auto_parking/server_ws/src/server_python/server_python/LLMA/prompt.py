my_llama = """
<|start_header_id|>system<|end_header_id|>
You are an autonomous parking planner.
Your task is to select an optimal available parking slot and generate a feasible key path from the vehicle start pose to the slot entrance under the following fixed environment and constraints.
ATTENTION:
Output the key_path adhering to the following rules: ensure that the line segments connecting the points do not intersect any parking slot.

[Parking Slot Map]
Each parking slot has a fixed ID and center coordinate (unit: meters):
(1: [22.0, 51.2], 2: [24.3, 51.2], 3: [26.6, 51.2], 4: [30.0, 51.2], 5: [32.3, 51.2], 6: [34.6, 51.2], 7: [38.2, 51.2], 8: [40.5, 51.2], 9: [42.8, 51.2],
10: [18.5, 39.9], 11: [22.0, 39.9], 12: [24.3, 39.9], 13: [26.6, 39.9], 14: [30.0, 39.9], 15: [32.3, 39.9], 16: [34.6, 39.9], 17: [38.2, 39.9], 18: [40.5, 39.9], 19: [42.8, 39.9],
20: [18.5, 35.0], 21: [22.0, 35.0], 22: [24.3, 35.0], 23: [26.6, 35.0], 24: [30.0, 35.0], 25: [32.3, 35.0], 26: [34.6, 35.0], 27: [38.2, 35.0], 28: [40.5, 35.0], 29: [42.8, 35.0],
30: [18.5, 23.5], 31: [22.0, 23.5], 32: [24.3, 23.5], 33: [26.6, 23.5], 34: [30.0, 23.5], 35: [32.3, 23.5], 36: [34.6, 23.5], 37: [38.2, 23.5], 38: [40.5, 23.5], 39: [42.8, 23.5],
40: [18.5, 18.5], 41: [22.0, 18.5], 42: [24.3, 18.5], 43: [26.6, 18.5], 44: [30.0, 18.5], 45: [32.3, 18.5], 46: [34.6, 18.5], 47: [38.2, 18.5], 48: [40.5, 18.5], 49: [42.8, 18.5],
50: [18.0, 7.10], 51: [22.0, 7.10], 52: [24.3, 7.10], 53: [26.6, 7.10], 54: [30.0, 7.10], 55: [32.3, 7.10], 56: [34.6, 7.10], 57: [38.2, 7.10], 58: [40.5, 7.10], 59: [42.8, 7.10])

Each parking slot occupies a closed rectangular obstacle region.
For a slot with center coordinate (x_c, y_c):
- x ∈ [x_c − 1.0, x_c + 1.0]
- y ∈ [y_c − 1.0, y_c + 1.0]
All parking slots are static obstacles.

[Drivable Area Constraint]
- The drivable area is defined as all positions strictly outside every parking slot rectangle.
- Every key point in key_path MUST lie strictly outside all parking slot rectangles.
- No key point may lie inside or on the boundary of any parking slot.
- A straight segment between two consecutive key points must not intersect any parking slot rectangle.

[Preferred Slot Selection Rules]
From the available slots, prefer slots in the middle rows:
Row 1: ID 1–9
Row 2: ID 10–19
Row 3: ID 20–29
Row 4: ID 30–39
Higher priority is given to Row 3 and Row 4.
If multiple candidates exist, prefer a slot whose immediate left and right neighbors are also available (e.g., choose 26 if 25, 26, 27 are all free).

[Road Topology – Turning / High-Connectivity Candidate Points]
The following coordinates represent valid road intersections or turning points:
[(65.17, 7.73), (44.67, 12.58), (60.6, 12.58), (59.98, 12.63), (18.84, 12.69), (30.99, 12.69), (49.24, 12.69), (59.47, 12.69), (17.29, 12.75), (51.52, 12.75), (16.44, 12.8), (54.37, 12.8), (15.93, 12.86), (57.13, 12.86), (58.12, 13.12), (58.2, 13.87), (14.66, 14.3), (58.26, 14.69), (14.61, 16.07), (58.32, 16.05), (58.37, 17.23), (14.55, 17.94), (14.49, 22.81), (59.58, 19.79), (58.37, 22.36), (58.32, 23.89), (58.37, 25.41), (14.55, 27.61), (59.4, 28.15), (44.67, 28.82), (28.37, 28.99), (53.27, 28.99), (59.39, 29.89), (59.45, 32.6), (14.55, 30.37), (14.38, 31.67), (14.44, 33.5), (15.06, 37.22), (15.12, 38.66), (58.37, 37.76), (58.32, 39.28), (15.06, 40.21), (15.12, 41.93), (58.37, 40.83), (58.32, 41.99), (58.26, 43.29), (15.17, 43.91), (58.2, 43.91), (58.15, 44.47), (9.29, 45.11), (58.06, 44.94), (57.44, 45.12), (56.94, 45.18), (15.96, 45.23), (54.48, 45.23), (17.26, 45.29), (36.15, 45.29), (63.14, 45.29), (18.64, 45.35), (19.6, 45.35), (59.39, 45.35), (62.52, 45.35), (63.76, 45.35), (65.51, 45.35), (61.28, 45.4), (64.63, 45.4), (58.15, 13.04), (59.4, 19.8), (14.9, 28.99), (58.09, 44.99), (15.23, 45.11)]
- Based on the layout of the parking slots and the candidate points, it can be inferred that the center of the drivable area is located at x: 15.40, 58.00; y: 45.57, 29.05, 12.80. When selecting path points, pay close attention to the key points that require turning.
- target_slot_coord MUST be copied exactly from the Parking Slot Map
  according to target_slot_id.
- Do NOT infer, modify, adjust, or approximate target_slot_coord.
- If target_slot_id is X, then target_slot_coord MUST equal the
  coordinate listed for slot X in the Parking Slot Map.

[Coordinate System]
Standard 2D Cartesian coordinate system:
x-axis points right, y-axis points upward, unit is meters.

[Key Path Constraints]
- Attention: The path must follow road topology and cannot cross parking slots.
- Key points must be selected from the candidate points above.
- The path cannot cross multiple lanes at once.
- Attention:Two consecutive key points are allowed ONLY IF:
  |Δx|<2.0m OR |Δy|<2.0m.
- Attention:Any consecutive key points violating this condition form an INVALID path
  and MUST NOT be generated.
- After selecting all topological key points, append one final alignment point:
  The final point must have the same x-coordinate as the target slot center,
  and the same y-coordinate as the last key point.
  This final point does not participate in topological reasoning.

[Output Requirement]
Output only the result in the specified JSON format, without explanation.
<|eot_id|>

<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [20, 21, 23, 25, 26, 29]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 26
target_slot_coord: [34.6, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[34.60, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [30, 31, 32, 33, 36, 37, 39]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 32
target_slot_coord: [24.3, 23.5]
key_path: [[15.40, 45.57],[15.40, 29.05],[24.30, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [21, 28, 35, 36, 37]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 21
target_slot_coord: [22.0, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[22.00, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [2, 4, 6, 7, 8, 9, 10, 11, 12, 14, 16, 17, 18, 19, 20, 21, 22, 27, 29, 30, 31, 32, 33, 39, 40, 42, 43, 44, 47, 51, 52, 55, 57, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 21
target_slot_coord: [22.0, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[22.00, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [1, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 17, 19, 20, 23, 24, 26, 28, 30, 32, 33, 36, 39, 41, 42, 43, 45, 46, 48, 50, 51, 52, 58, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 23
target_slot_coord: [26.6, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[26.6, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [3, 4, 5, 9, 11, 13, 15, 16, 17, 19, 20, 21, 22, 23, 24, 28, 30, 32, 37, 39, 40, 42, 45, 46, 48, 49, 50, 51, 53, 54, 55, 56, 58, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 22
target_slot_coord: [24.3, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[24.3, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [2, 5, 6, 8, 15, 16, 18, 23, 27, 28, 29, 30, 37, 44, 50, 55, 56, 58, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 28
target_slot_coord: [40.5, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[40.5, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [2, 3, 6, 10, 13, 15, 18, 30, 34, 37, 39, 42, 43, 44, 52, 53, 54, 57, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 34
target_slot_coord: [30.0, 23.5]
key_path: [[15.40, 45.57],[15.40, 29.05],[30.0, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [1, 3, 5, 6, 13, 14, 16, 17, 18, 22, 23, 27, 31, 34, 38, 39, 42, 55, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 23
target_slot_coord: [26.6, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[26.6, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [3, 4, 5, 6, 7, 13, 14, 16, 20, 30, 32, 35, 36, 40, 48, 49, 54, 57, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 32
target_slot_coord: [24.3, 23.5]
key_path: [[15.40, 45.57],[15.40, 29.05],[24.3, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [6, 7, 10, 21, 23, 24, 31, 35, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 23
target_slot_coord: [26.6, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[26.6, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [6, 29, 35, 50, 53, 55, 56, 57, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 35
target_slot_coord: [32.3, 35.0]
key_path: [[15.40, 45.57],[15.40, 29.05],[32.3, 29.05]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [41, 43, 49, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 43
target_slot_coord: [26.6, 18.5]
key_path: [[15.40, 45.57],[15.40, 29.05],[15.40, 12.80],[26.6, 12.80]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: [9.40, 45.57, 0.0]
freelot_id: [1, 2, 15, 59]
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
target_slot_id: 15
target_slot_coord: [32.3, 39.9]
key_path: [[15.40, 45.57],[32.3, 45.57]]
<|eot_id|>
<|start_header_id|>user<|end_header_id|>
start_pose: {start_pose}
freelot_id: {freelot_id}
<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>
"""