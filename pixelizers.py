import cv2.aruco as aruco
import cv2
import gym
import pix_main_arena
import time
import pybullet as py
import pybullet_data
import os
import numpy as np

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
board = aruco.GridBoard_create(
    markersX=2,
    markersY=2,
    markerLength=0.09,
    markerSeparation=0.01,
    dictionary=ARUCO_DICT)
rvecs, tvecs = None, None
n=12
direc=[0,0]
circles=[]
squares=[]
if __name__=="__main__":
    parent_path = os.path.dirname(os.getcwd())
    os.chdir(parent_path)
    env = gym.make("pix_main_arena-v0")
    time.sleep(3)
    range1 = np.zeros((n + 1,), dtype=int)
    for x in range(n + 1):
        range1[x] = 360 * x / n
    
    range2 = np.zeros((n + 1,), dtype=int)
    for x in range(n + 1):
        range2[x] = 720 * x / n

    range3 = np.zeros((n + 1,), dtype=int)
    for x in range(n + 1):
        range3[x] = 1200 * x / n
        
    x = 0
    y = 0
    inx = 1
    positive=1
    stop = np.array([30,90,150,210,270,330,390,450,510,570,630,675])


    j = 0
    ad=np.zeros((n*n,n*n),dtype=int)
    arr1=np.zeros((10,4),dtype=int)
    check=np.zeros((n,n), dtype=int)
    arr=np.array([[0,0,165,132,146,255],
                  [39,125,104,84,255,255],
                  [30,125,104,35,255,255],
                  [0,125,104,0,255,255],
                  [130,104,102,153,153,255],
                  [81,155,102,102,255,255],
                  [118,179,114,132,255,255]])

    env.remove_car()
    img = env.camera_feed()
    cv2.imwrite('i.jpg',img)

    frame1=cv2.imread('i.jpg')  

    frame1 = cv2.resize(frame1, (600, 600))

    frame = frame1[20:580, 20:580]
    frame = cv2.resize(frame, (1200, 1200))
    cv2.imwrite('igg.jpg', frame)

    env.respawn_car()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.medianBlur(hsv, 3)
    while j < 7:
        i = 0
        l_h = arr[j][0]
        l_s = arr[j][1]
        l_v = arr[j][2]
        u_h = arr[j][3]
        u_s = arr[j][4]
        u_v = arr[j][5]
        j = j + 1
        l_b = np.array([l_h, l_s, l_v])
        u_b = np.array([u_h, u_s, u_v])
        mask_color = cv2.inRange(hsv, l_b, u_b)
        det_color = cv2.bitwise_and(frame, frame, mask=mask_color)
        contours, _ = cv2.findContours(mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        cv2.waitKey(1)
        if (len(contours) >= 1 and j < 7):
            contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

            while i < len(contours):
                if (cv2.contourArea(contours[i]) > 50):
                    M = cv2.moments(contours[i])
                    if (M['m00'] != 0):
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        h = 0
                        k = 0
                        for x in range(n):
                            if (cx > range3[x] and cx < range3[x + 1]):
                                k = x
                            if (cy > range3[x] and cy < range3[x + 1]):
                                h = x
                        check[h][k] = j
                        if (j == 5):
                            check[h][k] = 35
                        if (j == 6):
                            check[h][k] = 36
                i = i + 1
        else:

            _, thrash = cv2.threshold(mask_color, 240, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            v = 0
            for c in contours:
                approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
                cv2.drawContours(frame, [approx], 0, (0, 0, 143), 5)
                cnt = sorted(approx, key=lambda x: cv2.contourArea(x), reverse=True)
                area = cv2.contourArea(c, oriented=False)
                peri = cv2.arcLength(c, closed=True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                h = 0
                k = 0
                for l in range(n):
                    if (x > range3[l] and x < range3[l + 1]):
                        k = l
                    if (y > range3[l] and y < range3[l + 1]):
                        h = l

                if int(peri * peri / area) < 15:
                    print(j, k, h, "circle")
                elif int(peri * peri / area) < 18:
                    print(j, k, h, "square")
                else:
                    arr1[v][0] = k
                    arr1[v][1] = h
                    x = np.zeros((3,), dtype=int)
                    y = np.zeros((3,), dtype=int)
                    for l in range(3):
                        x[l] = approx[l][0][0]
                        y[l] = approx[l][0][1]
                    check1 = 0
                    h=0
                    k=0

                    for l in range(3):
                        for m in range(3):
                            if (x[l] - x[m] < 4 and x[l] - x[m] > -4 and l != m):
                                h = l
                                k = m
                                check1 = 1
                            if (y[l] - y[m] < 4 and y[l] - y[m] > -4 and l != m):
                                h = l
                                k = m
                                check1 = 0
                    find = 0
                    if (check1 == 0):
                        arr1[v][2] = arr1[v][0]
                        if ((y[3 - h - k] - y[h]) < 0):
                            find = 1
                            arr1[v][3] = arr1[v][1] - 1
                        else:
                            find = 3
                            arr1[v][3] = arr1[v][1] + 1
                    if (check1 == 1):
                        arr1[v][3] = arr1[v][1]
                        if ((x[3 - h - k] - x[h]) < 0):
                            find = 4
                            arr1[v][2] = arr1[v][0] - 1
                        else:
                            find = 2
                            arr1[v][2] = arr1[v][0] + 1
                    v = v + 1


    for i in range(int(n/2)):
        for j in range(n):
            temp=check[n-1-i][n-1-j]
            check[n-1-i][n-1-j]=check[i][j]
            check[i][j]=temp
    for i in range(4):
        for j in range(4):
            arr1[i][j]=n-1-arr1[i][j]


    for i in range(n*n):
        p=i//n
        q=i%n
        if(i==0):
            ad[i][i+1]=check[p][q+1]
            ad[i][i+n]=check[p+1][q]
        elif(i==11):
            ad[i][i-1]=check[p][q-1]
            ad[i][i+n]=check[p+1][q]
        elif(i==132):
            ad[i][i+1]=check[p][q+1]
            ad[i][i-n]=check[p-1][q]
        elif(i==143):
            ad[i][i-1]=check[p][q-1]
            ad[i][i-n]=check[p-1][q]
        elif(i>=1 and i<=10):
            ad[i][i+1]=check[p][q+1]
            ad[i][i+n]=check[p+1][q]
            ad[i][i-1]=check[p][q-1]
        elif(i==12 or i==24 or i==36 or i==48 or i==60 or i==72 or i==84 or i==96 or i==108 or i==120):
            ad[i][i+1]=check[p][q+1]
            ad[i][i+n]=check[p+1][q]
            ad[i][i-n]=check[p-1][q]
        elif(i==23 or i==35 or i==47 or i==59 or i==71 or i==83 or i==95 or i==107 or i==119 or i==131):
            ad[i][i+n]=check[p+1][q]
            ad[i][i-1]=check[p][q-1]
            ad[i][i-n]=check[p-1][q]
        elif(i==133 or i==134 or i==135 or i==136 or i==137 or i==138 or i==139 or i==140 or i==141 or i==142):
            ad[i][i-1]=check[p][q-1]
            ad[i][i-n]=check[p-1][q]
            ad[i][i+1]=check[p][q+1]
        else:
            ad[i][i-1]=check[p][q-1]
            ad[i][i-n]=check[p-1][q]
            ad[i][i+1]=check[p][q+1]
            ad[i][i+n]=check[p+1][q]

    for i in range(4):
        if ((arr1[i][0]==11 and arr1[i][1]==11 and arr1[i][2]==11 and arr1[i][3]==11)!=1) :
            x= arr1[i][1]*n+arr1[i][0]
            y= arr1[i][3]*n+arr1[i][2]
            ad[y][x]=0
            if(arr1[i][0]==arr1[i][2]):
                if(2*x-y>=0 and 2*x-y<=143):
                    ad[x][2*x-y]=0
                if(arr1[i][0]>0):
                    ad[x][x-1]=0
                if(arr1[i][0]<11):
                    ad[x][x+1]=0
            if (arr1[i][1] == arr1[i][3]):
                if(arr1[i][0]>0 and arr1[i][0]<11):
                    ad[x][2*x-y]=0
                if(x-n>=0):
                    ad[x][x - n] = 0
                if(x+n<=n*n-1):
                    ad[x][x + n] = 0

    start = 0
    end=0
    check1=0
    listx=[]
    listy=[]
    minimum=0
    maxval=10000
    abc=1

    class Graph:
        start=0
        def minDistance(self, dist, queue):

            global n
            minimum = float("Inf")
            min_index = -1

            for i in range(len(dist)):
                if dist[i] < minimum and i in queue:
                    minimum = dist[i]
                    min_index = i
            return min_index

        def printPath(self, parent, j,dist):
            global maxval
            global end
            global abc
            global n
            global check1
            if(j==end and check1==0):
                check1 =1
                k=j
                if maxval>dist[j] or abc:
                    listx.clear()
                    listy.clear()
                    maxval=dist[j]
                    while(parent[k]!=-1):
                        listy.append(k//n)
                        listx.append(k%n)
                        k=parent[k]
    
            if parent[j] == -1:
                return
            self.printPath(parent, parent[j],dist)

        def printSolution(self, dist, parent):
            src = 0
            global n
            for i in range(1, len(dist)):
                self.printPath(parent, i,dist)

        '''Function that implements Dijkstra's single source shortest path
        algorithm for a graph represented using adjacency matrix
        representation'''

        def dijkstra(self, graph, src):
            global start
            global check1
            check1=0
            global n
            global end
            row=len(graph)
            col=len(graph[0])
            dist = [10000] * row
            parent = [-1] * row
            dist[start] = 0
            queue = []
            for i in range(row):
                queue.append(i)
            while queue:
                u = self.minDistance(dist,queue)
                if u<0 :
                    u=0
                queue.remove(u)
                for i in range(col):
                    if graph[u][i] and i in queue:
                        if dist[u] + graph[u][i] < dist[i]:
                            dist[i] = dist[u] + graph[u][i]
                            parent[i] = u
            
            self.printSolution(dist,parent)

    g= Graph()
    graph = ad
    def graphh():
        global maxval
        maxval=10000
        mini=0
        global abc
        global n
        global start
        global end
        start=end
        abc=0
        x=0
        for i in range(n):
            for j in range(n):
                if(check[i][j]==35):

                    end=x
                    g.dijkstra(graph,x)

                x=x+1
        abc=1
        end=listx[0] + n* listy[0]
        start=listx[len(listx)-1] + 12* listy[len(listy)-1]



#*************************************ARUCO....FORWARD...STOPNOW....LEFT....RIGHT****************************************************************88

    def aruconotdetect(x,y):
        print('not detected')
        global n
        h=0
        k=0
        r=0.0
        for q in range(n):
            if (x > range2[q] and x < range2[q+ 1]):
                k = q
            if (y > range2[q] and y < range2[q + 1]):
                h = q
        if(h==0):
            if(direc[0]>0):
                r=-0.5
            if(direc[0]<0):
                r=0.5
        if (h == n-1):
            if (direc[0] > 0):
                r=0.5
            if (direc[0] < 0):
                r=-0.5
        if (k == 0):
            if (direc[1] > 0):
                r=0.5
            if (direc[1] < 0):
                r=-0.5
        if (k == n-1):
            if (direc[1] > 0):
                r=-0.5
            if (direc[1] < 0):
                r=0.5

        x = 5
        start = time.time() * 10
        while True:
            py.stepSimulation()
            env.move_husky(r,-r,r,-r)
            if (int(time.time() * 10) == int(start) + x):
                break
            
        x = 20
        start = time.time() * 10
        while True:
            py.stepSimulation()
            env.move_husky(1.5,1.5,1.5,1.5)
            if (int(time.time() * 10) == int(start) + x):
                break
    
    def ar():
        global n
        img1 = env.camera_feed()
        img2 = cv2.resize(img1, (720,720))

        img = cv2.rotate(img2, cv2.ROTATE_90_CLOCKWISE)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('img2.jpg', img)
        corners1, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        global x
        global y
        global inx
        if ids is not None:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
            y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4

            x= 720/685 * (x-35)
            y= 720/685 * (y-35)

            direc[0]=corners[0][0][0][0]-corners[0][0][3][0]
            direc[1]=corners[0][0][0][1]-corners[0][0][3][1]

        else:
            aruconotdetect(x,y)

    def forward(num1, num2):
        global x
        global y
        global n
        global inx
        check = 1
        ar()
        a=direc[0]
        b=direc[1]
        while check:

            x=2
            start=time.time()*10
            while True:
                py.stepSimulation()
                env.move_husky(1.5,1.5,1.5,1.5)
                if(int(time.time()*10)==int(start)+x):
                    break
            ar()
            if (inx):
                if a>0:
                    if (x>stop[num1]-10):
                        check=0
                else:
                    if(x<stop[num1]-10):
                        check=0
            else:
                if b>0:
                    if (y > stop[num2]-10):
                        check=0
                else:
                    if(y<stop[num2]-10):
                        check=0

    def stopnow(x1, y1, corners):
        global inx
        global n
        x2 = corners[0][0][1][0]-corners[0][0][0][0]
        y2 = corners[0][0][1][1]-corners[0][0][0][1]

        if(x1*x2+y1*y2)==0:
            return 0
        if(inx==0):
            if(abs(corners[0][0][0][1]-corners[0][0][1][1])<4 or abs(corners[0][0][1][0]-corners[0][0][2][0])<4):
                return 0
        else:
            if(abs(corners[0][0][0][0]-corners[0][0][1][0])<4 or abs(corners[0][0][1][1]-corners[0][0][2][1])<4):
                return 0
        return 1

    def left():
        global n
        global inx
        if(inx):
            inx=0
        else:
            inx=1
        img = env.camera_feed()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners1, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        if ids is not None:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        if corners[0][0][0][0] == corners[0][0][1][0]:
            if corners[0][0][0][0]-corners[0][0][3][0] > 0:
                print("+ x")
            else:
                print("- X")
        if corners[0][0][0][1] == corners[0][0][1][1]:
            if corners[0][0][0][1]-corners[0][0][3][1] > 0:
                print("+ Y")
            else:
                print("- Y")

        x1 = corners[0][0][1][0]-corners[0][0][0][0]
        y1 = corners[0][0][1][1]-corners[0][0][0][1]

        while(stopnow(x1,y1,corners)):
            img = env.camera_feed()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            x=2

            start=time.time()*10
            while True:
                py.stepSimulation()
                env.move_husky(-0.4,0.4,-0.4,0.4)
                if(int(time.time()*10)==int(start)+x):
                    break
        ar()
        if((abs(direc[0])>=2 and abs(direc[0])<=4) or (abs(direc[1])>=2 and abs(direc[1])<=4)):
            x = 4
            start = time.time() * 10
            while True:
                py.stepSimulation()
                env.move_husky(-0.2, 0.2, -0.2, 0.2)
                if (int(time.time() * 10) == int(start) + x):
                    break


    def right():
        global n
        global inx
        if(inx):
            inx=0
        else:
            inx=1
        img = env.camera_feed()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners1, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        if ids is not None:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

        if corners[0][0][0][0] == corners[0][0][1][0]:
            if corners[0][0][0][0]-corners[0][0][3][0] > 0:
                print("+ x")
            else:
                print("- X")
        if corners[0][0][0][1] == corners[0][0][1][1]:
            if corners[0][0][0][1]-corners[0][0][3][1] > 0:
                print("+ Y")
            else:
                print("- Y")

        x1 = corners[0][0][1][0]-corners[0][0][0][0]
        y1 = corners[0][0][1][1]-corners[0][0][0][1]

        while(stopnow(x1,y1,corners)):
            
            img = env.camera_feed()

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            x=2
            start=time.time()*10
            while True:
                py.stepSimulation()
                env.move_husky(0.4,-0.4,0.4,-0.4)
                if(int(time.time()*10)==int(start)+x):
                    break

        ar()
        if((abs(direc[0])>=2 and abs(direc[0])<=4) or (abs(direc[1])>=2 and abs(direc[1])<=4)):
            x = 4
            start = time.time() * 10
            while True:
                py.stepSimulation()
                env.move_husky(0.2, -0.2, 0.2, -0.2)
                if (int(time.time() * 10) == int(start) + x):
                    break
#**************************************************PINK***************************************************************

    def pink():
        global n
        global start 
        global end
        global tellme
        circles.clear()
        squares.clear()
        x = 25
        start1 = time.time() * 10
        while True:
            py.stepSimulation()
            env.move_husky(-0.2,-0.2,-0.2,-0.2)
            if (int(time.time() * 10) == int(start1) + x):
                break
        x=0
        while x<2:
            py.stepSimulation()
            env.remove_cover_plate(11-listy[len(listx)-1],11-listx[len(listy)-1])
            img = env.camera_feed()
            cv2.imwrite('i.jpg',img)
            x+=1

        frame1 = cv2.imread('i.jpg')
        frame1 = cv2.resize(frame1, (600,600))
        frame = frame1[20:580, 20:580]
        frame = cv2.resize(frame, (1200, 1200))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv, 3)

        l_h = arr[6][0]
        l_s = arr[6][1]
        l_v = arr[6][2]
        u_h = arr[6][3]
        u_s = arr[6][4]
        u_v = arr[6][5]
        # j = j + 1
        l_b = np.array([l_h, l_s, l_v])
        u_b = np.array([u_h, u_s, u_v])
        mask_color = cv2.inRange(hsv, l_b, u_b)
        det_color = cv2.bitwise_and(frame, frame, mask=mask_color)

        _, thrash = cv2.threshold(mask_color, 240, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        v = 0
        for c in contours:
            approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c,True), True)
            cv2.drawContours(frame, [approx], 0, (0, 0, 143), 5)
            cnt = sorted(approx, key=lambda x: cv2.contourArea(x), reverse=True)
            area = cv2.contourArea(c, oriented=False)
            peri = cv2.arcLength(c, closed=True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            h = 0
            k = 0
            for l in range(n):
                if (x > range3[l] and x < range3[l + 1]):
                    k = l
                if (y > range3[l] and y < range3[l + 1]):
                    h = l

            if int(peri*peri/area) < 15:
                circles.append(n*(n-1-h)+(n-1-k))
                print(j, k, h, "circle")
            elif int(peri*peri/area) < 18:
                squares.append(n*(n-1-h)+(n-1-k))
                print(j, k, h, "square")
        start=end

        if len(circles)==2:
            if(circles[0]==start):
                end=circles[1]
            if(circles[1]==start):
                end=circles[0]
        if(len(squares)==2):
            if(squares[0]==start):
                end=squares[1]
            if(squares[1]==start):
                end=squares[0]


#************************************************FIND COORDINATES....DRIVER*******************************************************************
    h1=0
    k1=0
    def findcoordinate():
        global n
        global x
        global y
        global h1
        global k1
        for l in range(n):
            if (x > range2[l] and x < range2[l + 1]):
                h1 = l
            if (y > range2[l] and y < range2[l + 1]):
                k1 = l

    def driver():
        global n
        positive=1
        global x
        global y
        print(listx)
        print(listy)
        ar()
        findcoordinate()
        i = 0
        while i < len(listx):
            if (inx):
                if(k1!=listy[i]):
                    if(i==0):
                        i=i+1
                    else:
                        positive = listx[i-1] - h1
                    forward(listx[i-1], listy[i - 1])
                    if (k1 < listy[i]):
                        if (positive > 0):
                            right()
                        else:
                            left()
                    else:
                        if (positive > 0):
                            left()
                        else:
                            right()
                    i=i-1
                    ar()
                    findcoordinate()
            else:
                if(h1!=listx[i]):
                    if(i==0):
                        i=i+1
                    else:
                        positive = listy[i - 1] - k1
                    forward(listx[i - 1], listy[i-1])
                    if (h1 < listx[i]):
                        if (positive > 0):
                            left()
                        else:
                            right()
                    else:
                        if (positive > 0):
                            right()
                        else:
                            left()
                    ar()
                    findcoordinate()
                    i=i-1
            if(i==len(listx)-1):
                forward(listx[i-1],listy[i-1])
            i=i+1

    #*******************************************EXECUTE.......ROTATE********************************************************

    def execute():
        global h1
        global k1
        global n
        r = 0.0
        if (h1==0 and k1==0):
            if (direc[0]>=0 and direc[1]>=0):
                r = 0.2
            else:
                r = -0.2
        if (h1==0 and k1==n-1):
            if (direc[0]>=0 and direc[1]<=0):
                r = 0.2
            else:
                r = -0.2
        if (h1==n-1 and k1==0):
            if (direc[0]<=0 and direc[1]>=0):
                r = 0.2
            else:
                r = -0.2
        if (h1==n-1 and k1==n-1):
            if (direc[0]<=0 and direc[1]<=0):
                r = 0.2
            else:
                r = -0.2
        x = 10
        start = time.time() * 10
        while True:
            py.stepSimulation()
            env.move_husky(r, r, r, r)
            if (int(time.time() * 10) == int(start) + x):
                break
    def rotate():
        global n
        global h1
        global k1
        global inx

        ar()
        findcoordinate()
        execute()
        ar()
        diffx=listx[0]-h1
        diffy=listy[0]-k1
        if(diffx==0):
            while ((direc[1]*diffy)<0 or abs(direc[0])>=5):
                x = 2
                start = time.time() * 10
                while True:
                    py.stepSimulation()
                    env.move_husky(0.5, -0.5, 0.5, -0.5)
                    if (int(time.time() * 10) == int(start) + x):
                        break
                ar()
        else:
            while ((direc[0]*diffx)<0 or abs(direc[1])>=5):

                x = 2
                start = time.time() * 10
                while True:
                    py.stepSimulation()
                    env.move_husky(0.5, -0.5, 0.5, -0.5)
                    if (int(time.time() * 10) == int(start) + x):
                        break
                ar()
        if abs(direc[0]) > 5:
            inx=1
        if abs(direc[1]) > 5:
            inx=0
        
        if((abs(direc[0])>=2 and abs(direc[0])<=5) or (abs(direc[1])>=2 and abs(direc[1])<=5)):
            x = 6
            start = time.time() * 10
            while True:
                py.stepSimulation()
                env.move_husky(0.2, -0.2, 0.2, -0.2)
                if (int(time.time() * 10) == int(start) + x):
                    break



    #**********************************************STEP UP********************************************************************

    def stepup():
        ar()
        a=direc[0]
        b=direc[1]
        global inx 
        global x
        global y
        global n
        chek=1
        while chek:
            py.stepSimulation()
            x1=2
            start=time.time()*10
            while True:
                py.stepSimulation()
                env.move_husky(1,1,1,1)
                if(int(time.time()*10)==int(start)+x1):
                    break
            if (inx):
                if a>0:
                    if (x + 20> 720/n*listx[len(listx)-1]):
                        chek=0
                else:
                    if(x < 20 + 720/n*listx[len(listx)-1]):
                        chek=0
            else:
                if b>0:
                    if (y + 10 > 720/n*listy[len(listy)-1]):
                        chek=0
                else:
                    if(y < 10 + 720/n*listy[len(listy)-1]):
                        chek=0
            ar()
            

    #****************************************************MAIN HEAD*******************************************************************
    def main_head():
        global n
        global x
        global y
        global h1
        global k1
        global end
        global start
        global inx
                                     #1st patient
        ar()
        findcoordinate()

        graphh()
        print(arr1)
        listx.reverse()
        listy.reverse()
        rotate()

        driver()
        pink()
        time.sleep(2)
        ar()
        stepup()
        check[listy[len(listx)-1]][listx[len(listy)-1]]=0
        
        print(check)
    #*************************************#
        listx.clear()                        #1st hospital
        listy.clear()
        g.dijkstra(graph,end)
        listx.reverse()
        listy.reverse()
        ar()
        rotate()
        driver()
        time.sleep(2)
        ar()
        stepup()

        listx.clear()           #2nd patient
        listy.clear()
        graphh()
        listx.reverse()
        listy.reverse()
        ar()
        rotate()
        driver()
        time.sleep(2)
        pink()
        ar()
        stepup()

        listx.clear()       #2nd hospital
        listy.clear()
        g.dijkstra(graph,end)
        listx.reverse()
        listy.reverse()
        ar()
        rotate()
        driver()
        time.sleep(2)
        ar()
        stepup()

    main_head()