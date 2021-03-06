import cv2
import math
import numpy as np
from cv2 import aruco
from numpy.linalg import inv
from square import square

def world2image(point3D,focalX,focalY,centerX,centerY,sX,sY):
    row = (point3D[0]*(focalX*sX)/point3D[2])
    row /= sY
    row = int(-(row-centerY))
    col = (point3D[1]*(focalY*sX)/point3D[2])
    col /= sX
    col = int(-(col-centerX))
    return (row,col)
sX = 0.0000025
sY = 0.0000033

fX2Pixel = 619.4993
fY2Pixel = 620.8682
cX2Pixel = 311.6270
cY2Pixel = 277.3101

k1 = 0.1287
k2 = -0.2772
k3 = 0.1752
p1 = -0.0007
p2 = -0.0064

cameraMatrix = np.asarray([
    [fX2Pixel ,0        ,cX2Pixel ],
    [0        ,fY2Pixel ,cY2Pixel ],
    [0        ,0        ,1        ]
])
distCoeffs = np.asarray([k1 ,k2 ,p1 ,p2 ,k3])
fX2Meter = fX2Pixel * sX
fY2Meter = fY2Pixel * sY
cX2Meter = cX2Pixel * sX
cY2Meter = cY2Pixel * sY

arucoLength = 0.029
cap = cv2.VideoCapture(0)
while (cap.isOpened()):
    _, tempMat = cap.read()
    h,  w = tempMat.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(w,h),1,(w,h))
    undistorted = cv2.undistort(tempMat,cameraMatrix,distCoeffs,newcameramtx)
    print('type of undistorted: ',type(undistorted))
    cv2.imshow("undistorted",undistorted)  
    gray = cv2.cvtColor(tempMat, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(tempMat.copy(), corners, ids)
    pointA=(0.0720401,-0.0127199,0.278)
    pointB=(-0.109992,0.0291681,0.278)
    color=(0,255,0)
    row,col = world2image(pointA,fX2Pixel,fY2Pixel,cX2Pixel,cY2Pixel,sX,sY)
    centerCoordinate1 = (int(col),int(row))
    row,col = world2image(pointB,fX2Pixel,fY2Pixel,cX2Pixel,cY2Pixel,sX,sY)
    centerCoordinate2 = (int(col),int(row))
    cv2.circle(frame_markers,centerCoordinate1,2,color, thickness=1, lineType=8, shift=0)
    cv2.circle(frame_markers,centerCoordinate2,2,color, thickness=1, lineType=8, shift=0)
    # print('image coordinate : ',centerCoordinate)
    cv2.imshow("arucoes drawed",frame_markers)
    arucoList = []
    if ids is not None:
        for i in range(len(ids)):
            corner2Pix = corners[i][0]
            corner2metre = np.zeros((4,2))
            corners3D = np.zeros((4,3))
            kc = np.zeros((4,2))
            for row in range(0,4):
                corner2metre[row][0] = (corner2Pix[row][0] * sX) - cX2Meter
                corner2metre[row][1] = (corner2Pix[row][1] * sY) - cY2Meter
                kc[row][0] = corner2metre[row][0] / fX2Meter
                kc[row][1] = corner2metre[row][1] / fY2Meter
            Q = [
                [kc[1][0] ,kc[3][0] ,-kc[0][0] ],
                [kc[1][1] ,kc[3][1] ,-kc[0][1] ],
                [    1    ,    1    ,    -1    ]
            ]
            Qinv = np.linalg.inv(Q)
            L0 = [kc[2][0] ,kc[2][1] ,1]
            L = np.zeros(3)
            for j in range(0,3):
                for k in range(0,3):
                    L[j] += Qinv[j][k] * L0[k]
            M = math.pow((kc[1][0] * L[0]) - (kc[0][0] * L[2]) ,2) + math.pow(kc[1][1] * L[0] - kc[0][1] * L[2] ,2) + math.pow(L[0] - L[2] ,2)
            corners3D[2][2] = arucoLength / math.sqrt(M)
            corners3D[0][2] = L[2] * corners3D[2][2]
            corners3D[1][2] = L[0] * corners3D[2][2]
            corners3D[3][2] = L[1] * corners3D[2][2]
            
            for cornerNum in range(0,4):
                corners3D[cornerNum][0] = kc[cornerNum][0] * corners3D[cornerNum][2]
                corners3D[cornerNum][1] = kc[cornerNum][1] * corners3D[cornerNum][2]
            # center = np.zeros((0,0))
            # center3D = np.zeros((0,0,0))
            arucoList.append(square(corner2metre,corners3D,arucoLength,kc,ids[i]))
            print(corners3D)
    cv2.waitKey(100)
