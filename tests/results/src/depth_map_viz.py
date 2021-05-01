import open3d as o3d
import cv2
import numpy as np

# folder_path = "/home/jesse/Code/src/ros/src/rostk_plotting/records/03:22:2021-09:21:49/"
folder_path = "/home/jesse/Code/src/ros/src/rostk_plotting/records/05:01:2021-10:16:07/"
# color_raw = o3d.io.read_image(folder_path + "0.png")
color_raw_numpy = cv2.imread(folder_path + "0.png")
color_raw_numpy = cv2.cvtColor(color_raw_numpy, cv2.COLOR_BGR2RGB)

depth_raw_numpy = cv2.imread(folder_path + "1.png")


# depth_raw = o3d.io.read_image(folder_path + "20_prediction.png")
# cv2.imshow("Depth", depth_raw_numpy)
# cv2.waitKey(1)

# depth_raw_numpy = cv2.normalize(src=depth_raw_numpy, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
depth_raw_numpy = cv2.bitwise_not(depth_raw_numpy)

# cv2.imshow("Depth inverted", depth_inverted)
# cv2.waitKey(-1)
# depth_inverted = depth_inverted.astype('uint8')
color_raw = o3d.geometry.Image(color_raw_numpy)
depth_raw = o3d.geometry.Image(depth_raw_numpy)

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, depth_trunc=1000, convert_rgb_to_intensity=False)

print(rgbd_image)


K = np.array([[392.31, 0, 320.75],
    [0, 467.9, 243.46],
    [0, 0, 1]])

D = np.array([-0.05400957120448697, -0.07842753582468161, 0.09596410068935728, -0.05152529532743679])
image_size = (640, 480)
R = np.eye(3,3)

P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(	K, D, image_size, R)


fx_new = P[0][0]
cx_new = P[0][2]
fy_new = P[1][1]
cy_new = P[1][2]

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(640, 480, fx_new, fy_new, cx_new, cy_new))


print(pcd)
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
o3d.visualization.ViewControl.set_zoom(vis.get_view_control(), 1)
vis.run()