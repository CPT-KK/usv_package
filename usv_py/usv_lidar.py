#!/usr/bin/env python3

import message_filters
from sensor_msgs.msg import PointCloud2

import open3d as o3d
import numpy as np
import itertools

from pctype import pointcloud22open3d
from usv_math import rotationMatrixToEulerAngles, rotationZ

class Lidar():
    # 时间戳
    t = 0 

    # 激光雷达探测到物体的总个数
    objNum = 0

    # 激光雷达探测到物体的 xy 坐标（船体系下），估计长、宽和朝向，以及船-物体视线角
    objInfo = np.zeros((20,6)) # 最多考虑 20 个物体

    # 目标船的预测范围
    tvPredictR = 15

    def __init__(self):
        self.lidar1 = message_filters.Subscriber('/usv/slot0/points', PointCloud2)
        self.lidar2 = message_filters.Subscriber('/usv/slot2/points', PointCloud2)
        self.lidar3 = message_filters.Subscriber('/usv/slot3/points', PointCloud2)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar1, self.lidar2, self.lidar3], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.lidarCallback)

    def cloudCut(self, cloud, xLB, xUB, yLB, yUB):
        if cloud.is_empty():
            return cloud
        
        # 定义 xyz 边界范围
        bounds_list = [[xLB, xUB], [yLB, yUB], [-10, 40]] 

        # 构造裁切盒 
        bbox_pt_list = list(itertools.product(*bounds_list))
        bbox_pt_vec = o3d.utility.Vector3dVector(bbox_pt_list)  # list
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_pt_vec)

        # 裁切点云
        cloudCrop = cloud.crop(bbox)

        # 可视化点云
        # o3d.visualization.draw(cloudCrop)

        return cloudCrop
                
    def cloudFilter(self, cloud):
        if cloud.is_empty():
            return cloud
        
        numNeighbors = 20 # K邻域点的个数
        stdRatio = 2.0 # 标准差乘数

        # 执行统计滤波，返回滤波后的点云sorCloud和对应的索引ind
        [sorCloud, ind] = cloud.remove_statistical_outlier(numNeighbors, stdRatio)
 
        # 提取点云
        # print("统计滤波后的点云：", sorCloud)
        # sorCloud.paint_uniform_color([0, 0, 1])

        # 提取噪声点云
        # sorNoiseCloud = cloud.select_by_index(ind,invert = True)
        # print("噪声点云：", sorNoiseCloud)
        # sorNoiseCloud.paint_uniform_color([1, 0, 0])

        # 可视化统计滤波后的点云和噪声点云
        # o3d.visualization.draw_geometries([sorCloud, sorNoiseCloud])

        return sorCloud
    
    def cloudTransform(self, cloud, dx, dy, dz, r, p, y):
        if cloud.is_empty():
            return cloud
        
        # 注意，这里的变换是转点，不是转坐标系

        # 平移：移回船的中心
        cloud = cloud.translate((dx, dy, dz))   

        # 旋转：与船体坐标系对齐
        rotMat = cloud.get_rotation_matrix_from_xyz([p, r, y])
        cloud = cloud.rotate(rotMat, (0, 0, 0))     

        return cloud

    def cloudCluster(self, cloud):
        if cloud.is_empty():
            return [], 0
        
        eps = 12           # 同一聚类中最大点间距
        minPoints = 40     # 有效聚类的最小点数

        # DBSCAN 聚类分割
        labels = np.array(cloud.cluster_dbscan(eps, minPoints, print_progress=False))
        # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                   
        # 获取聚类标签的最大值 [-1,0,1,2,...,max_label]，label = -1 为噪声，因此总聚类个数为 max_label + 1
        clusterNum = labels.max() + 1

        # 可视化所有聚类点云：随机构建n+1种颜色，并归一化
        # colors = np.random.randint(1, 255, size=(clusterNum, 3)) / 255.
        # colors = colors[labels]             # 每个点云根据label确定颜色
        # colors[np.array(labels) < 0] = 0    # 噪点配置为黑色
        # cloud.colors = o3d.utility.Vector3dVector(colors)
        # o3d.visualization.draw_geometries([cloud], window_name="cluster", width=800, height=600)

        clusters = []
        for i in range(clusterNum + 1):
            idx = np.where(labels == i)[0]
            clusters.append(cloud.select_by_index(idx))

        return clusters, clusterNum
    
    def lidarCallback(self, pc1, pc2, pc3):
        # 点云转换
        self.t = (pc1.header.stamp.sec + 1e-9 * pc1.header.stamp.nanosec + pc2.header.stamp.sec + 1e-9 * pc2.header.stamp.nanosec + pc3.header.stamp.sec + 1e-9 * pc3.header.stamp.nanosec) / 3.0
        cloud1 = pointcloud22open3d(pc1)
        cloud2 = pointcloud22open3d(pc2)
        cloud3 = pointcloud22open3d(pc3)

        # 点云处理
        cloud1 = self.cloudCut(cloud1, -150, 150, -150, 150)
        cloud1 = self.cloudFilter(cloud1)
        cloud1 = self.cloudTransform(cloud1, 0.738, 0, 1.68, 0, 0, 0)
        
        cloud2 = self.cloudCut(cloud2, 0, 150, -150, 150)
        cloud2 = self.cloudFilter(cloud2)
        cloud2 = self.cloudTransform(cloud2, 1.7, 0, 1.14, 0, 0, np.pi/2)

        cloud3 = self.cloudCut(cloud3, 0, 150, -150, 150)
        cloud3 = self.cloudFilter(cloud3)
        cloud3 = self.cloudTransform(cloud3, 1.7, 0, 1.14, 0, 0, -np.pi/2)     

        # 点云合并，并计算聚类数
        cloudSum = cloud1 + cloud2 + cloud3
        [clusters, clusterNum] = self.cloudCluster(cloudSum)

        self.objNum = clusterNum
        self.objInfo = np.zeros((20,6))

        if clusterNum <= 0:
            return

        for i in range(clusterNum):
            if i > 19:
                break
               
            # 聚类求 obb        
            # obb = clusters[i].get_oriented_bounding_box(robust=True)
            obb = clusters[i].get_oriented_bounding_box(robust=True)

            # 由旋转矩阵计算聚类的朝向角
            [ex, ey, ez] = rotationMatrixToEulerAngles(obb.R)
            
            # 计算聚类的相对 xy 坐标，长宽 lw 以及其在船体坐标系下的朝向角
            x = obb.center[0]
            y = obb.center[1]
            l = obb.extent[0]
            w = obb.extent[1]
            heading = np.arctan(np.tan(ez))

            # 保存
            self.objInfo[i, 0] = x
            self.objInfo[i, 1] = y
            self.objInfo[i, 2] = l
            self.objInfo[i, 3] = w
            self.objInfo[i, 4] = heading
            self.objInfo[i, 5] = np.arctan2(y, x)

            # 输出聚类 xylwh 的信息
            # print("Cluster %d:\t at [%.2fm, %.2fm] \t| len %.2fm, wid %.2fm \t| heading %.2fdeg" % (i + 1, x, y, l, w, np.rad2deg(heading)))

            # 输出聚类数目
            # print(f"Point cloud has {clusterNum} cluster(s)")

            # 可视化每个聚类
            # o3d.visualization.draw_geometries([clusters[i]], window_name="cluster", width=800, height=600)

        # 可视化总点云
        # cloud1.paint_uniform_color([1, 0, 0])
        # cloud2.paint_uniform_color([0, 1, 0])
        # cloud3.paint_uniform_color([0, 0, 1])
        # o3d.visualization.draw_geometries([cloud1, cloud2, cloud3])
        
    def objRead(self, x, y, psi, beta, tvEstX, tvEstY):
        idxTV = None
        idxObs = None
        isTVFound = False
        isObsFound = False

        # 激光雷达识别至少一个物体时，才进行判断
        if (self.objNum > 0):    
            # 获取目标船在无人船船体系下的坐标
            [dx, dy] = rotationZ(tvEstX - x, tvEstY - y, psi)
            
            # 获取 激光雷达扫描到的物体 与 目标船（在无人船船体系下） 的距离
            # 获取 激光雷达扫描到的物体 与 无人船 的距离
            dist2EstTV = np.linalg.norm(np.dot(np.ones((self.objNum, 1)), np.array([[dx, dy]])) - self.objInfo[0:self.objNum, 0:2], axis=1) # 2 个 [] 对应的才是向量
            dist2USV = np.linalg.norm(self.objInfo[0:self.objNum, 0:2], axis=1)

            # 获取 dist2EstTV 中的最小值，判断其是否为目标船，如果是，则记录其索引，并在 dist2USV 中将目标船的距离设为 inf，避免其被识别为障碍物
            if np.min(dist2EstTV) <= self.tvPredictR:
                idxTV = np.argmin(dist2EstTV)
                isTVFound = True
                dist2USV[idxTV] = np.Inf

            # 取 dist2USV 中的最小值，判断其是否为障碍物
            # 障碍物为非目标船，且距离船在 60m 内的最近的那一个，且在速度方向的正负 60 deg (1.0471976 rad) 内
            if (np.min(dist2USV) <= 60.0) & (abs(self.objInfo[np.argmin(dist2USV), 5] - beta) <= 1.0471976):
                idxObs = np.argmin(dist2USV)
                isObsFound = True
            
        return [idxTV, isTVFound, idxObs, isObsFound]
        
