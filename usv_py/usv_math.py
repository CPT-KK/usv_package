from numpy import pi, sign, arctan2, sin, cos, abs, array, sqrt

def wrapToPi(x):
    x = arctan2(sin(x), cos(x))
    return x

def wrapTo2Pi(x):
    x = wrapToPi(x)
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

def rotationZ(x,  y,  angle):
    xNew = x * cos(angle) + y * sin(angle)
    yNew = -x * sin(angle) + y * cos(angle)

    return [xNew, yNew]

# checklist = deg2rad(array([10, 20, 30, 45, 60, 89, 90, 91, 135, 150, 179, 181, 200, 215, 260, 270, 359, 361]))
# print(rad2deg(wrapToPi(checklist)))
# print(rad2deg(wrapToPi(-checklist)))
# for i in checklist:
#     print("%.2f" % (wrapToPi(i)))
    # print(wrapToPi(-i))