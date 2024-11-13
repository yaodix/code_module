# python3.8
import numpy as np
import cv2
import matplotlib
import matplotlib.pyplot as plt
from pathlib import Path
import sys
sys.path.append(str(Path('../../')))
from solutions.camera_calibration.calibrated_lane_detector import CalibratedLaneDetector, get_intersection, get_py_from_vp

fig, ax = plt.subplots()  
# load an image for which the yaw angle was set to 2 degrees and the pitch angle was set to to -3 degrees in the Carla simulator
image_fn = str(Path("../../../data/Image_yaw_2_pitch_-3.png"))
image = cv2.imread(image_fn)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
ax.imshow(image)
model_path = Path("../../solutions/lane_detection/fastai_model.pth")
cld = CalibratedLaneDetector(model_path=model_path)
def plot_detected_lines(line_left, line_right):
    u = np.arange(0,cld.cg.image_width, 1)
    v_left = line_left(u)
    v_right = line_right(u)

    ax.plot(u,v_left, color='r')
    ax.plot(u,v_right, color='b')
_, left_probs, right_probs = cld.detect(image)
line_left  = cld._fit_line_v_of_u(left_probs)
line_right = cld._fit_line_v_of_u(right_probs)
plot_detected_lines(line_left, line_right)

# valishing point near (469, 191)
vanishing_point = get_intersection(line_left, line_right)
print(vanishing_point)
u_i, v_i = vanishing_point
ax.scatter([u_i],[v_i], marker="o", s=100, color="c", zorder=10)
pitch, yaw = get_py_from_vp(u_i, v_i, cld.cg.intrinsic_matrix)
# print values and compare to the expected result
print("pitch (deg):\n  Computed:   {:.2f}\n  True value: {:.2f}".format(np.rad2deg(pitch), -3.00))
print("yaw (deg):\n  Computed:   {:.2f}\n  True value: {:.2f}".format(np.rad2deg(yaw), 2.00))
print("")
plt.show()