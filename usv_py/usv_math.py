#!/usr/bin/env python3
from usv_constants import *
from numpy import pi, sign, arctan2, sin, cos, abs, array, sqrt, percentile
from sklearn.cluster import DBSCAN
from collections import Counter

def wrapToPi(x):
    x = arctan2(sin(x), cos(x))
    return x

def wrapTo2Pi(x):
    x = array(wrapToPi(x))
    x[x < 0] = x[x < 0] + 2 * pi

    return x

def rotationMatrixToEulerAngles(R) :
    # 奇异性判断
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])    
    singular = sy < 1e-6
 
    if not singular :
        x = arctan2(R[2,1] , R[2,2])
        y = arctan2(-R[2,0], sy)
        z = arctan2(R[1,0], R[0,0])
    else :
        x = arctan2(-R[1,2], R[1,1])
        y = arctan2(-R[2,0], sy)
        z = 0
 
    return array([x, y, z])

def rotationZ(x, y, angle):
    xNew = x * cos(angle) + y * sin(angle)
    yNew = -x * sin(angle) + y * cos(angle)

    return [xNew, yNew]

def linearClip(x1, y1, x2, y2, x):
    # 线性插值并且限幅
    # x in [x1, x2]，则线性插值 y
    # x < x1，y = y1
    # x > x2，y = y2

    # 判断 x1 x2 哪个大
    if (x1 > x2):
        [x1, x2, y1, y2] = [x2, x1, y2, y1]

    if (x > x2):
        y = y2
    elif (x < x1):
        y = y1
    else:
        y = (y2 - y1) / (x2 - x1) * (x - x1) + y1

    return y

# def removeOutliers(data):
#     Q1 = percentile(data, 25)
#     Q3 = percentile(data, 75)
#     IQR = Q3 - Q1
#     lb = Q1 - 1.5 * IQR
#     ub = Q3 + 1.5 * IQR
    
#     return data[(data >= lb) & (data <= ub)]

def removeOutliers(data, eps, min_samples):
    # 两个点是邻居的最大距离 eps
    # 一个点被认为是核心点的最小邻居数目 min_samples
    data = data.reshape(-1, 1)  # 转换为二维数组，因为DBSCAN需要二维输入
    clustering = DBSCAN(eps=eps, min_samples=min_samples ,n_jobs=-1).fit(data)
    cluster_labels = clustering.labels_

    # 找到数量最大的簇的标签
    cluster_counts = Counter(cluster_labels)
    most_common_label = cluster_counts.most_common(1)[0][0]

    # 返回数量最大簇的数据
    return data[cluster_labels == most_common_label].flatten()  # -1标签对应的是离群值


# checklist = deg2rad(array([10, 20, 30, 45, 60, 89, 90, 91, 135, 150, 179, 181, 200, 215, 260, 270, 359, 361]))
# print(rad2deg(wrapToPi(checklist)))
# print(rad2deg(wrapToPi(-checklist)))
# for i in checklist:
#     print("%.2f" % (wrapToPi(i)))
    # print(wrapToPi(-i))

def updateTVHeading(existHeading, newHeading):
    newHeading2 = wrapToPi(newHeading + pi)
    angleGap1 = abs(wrapToPi(newHeading - existHeading))
    angleGap2 = abs(wrapToPi(newHeading2 - existHeading))

    if (angleGap1 <= angleGap2):
        return wrapToPi(newHeading)
    else:
        return wrapToPi(newHeading2)

def calcHighest(tvHighestXMean, tvHighestYMean, tvHighestZMean, tvLengthMean, tvYaw):
    if (tvHighestZMean >= HEALTHY_Z_TOL):   
        # 最高点测量健康，向最高点映射到船中轴线上的点泊近
        # 注意：这里的 rotationZ 是要对点（向量）进行旋转，即求取点在旋转后的坐标（同一坐标系下），
        # 而不是同一个点在不同坐标系下的表示，故取负号
        [tvHighestXMean2, _] = rotationZ(tvHighestXMean, tvHighestYMean, tvYaw)
        if (tvHighestXMean2 >= 0):
            xf = (-0.5 * tvLengthMean + tvHighestXMean2) / 2
            yf = 0.0
        else:
            xf = (0.5 * tvLengthMean + tvHighestXMean2) / 2
            yf = 0.0
        [xf, yf] = rotationZ(xf, yf, -tvYaw)
    else: 
        # 最高点测量不健康，设置为目标船中心
        xf = 0.0
        yf = 0.0

    return [xf, yf]
