# -*- coding: utf-8 -*-

"""Pattern recognition

This file provides functions to find well-defined patterns in an image
"""
import cv2

def logFileWrite(file,msg):
    pass
#        file.write("%s,%s\n" % (str(time.time()), msg))

def cornerPointsChess(img,logFile):
    """Find chessboard in image
    
    Args:
        img: An openCV image ndarray in a grayscale or color format.
        logFile: File handler for logfile.
    """
    NBR_COLUMNS = 3
    NBR_ROWS = 3
    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    ret, corners = cv2.findChessboardCorners(gray, (NBR_COLUMNS,NBR_ROWS),None)
    
    xalt=[]
    yalt=[]
    sumV=[]
    
    if ret == True:
#        print "Chessboard found!"
        logFileWrite(logFile,"Chessboard found!")
        cv2.imwrite('testewr.png',img)
        #Find left-top corner value && right-bottom corner value
        xalt.append(round(corners[0][0][0]))
        yalt.append(round(corners[0][0][1]))
        xalt.append(round(corners[NBR_COLUMNS-1][0][0]))
        yalt.append(round(corners[NBR_COLUMNS-1][0][1]))
        xalt.append(round(corners[NBR_COLUMNS*NBR_ROWS-1][0][0]))
        yalt.append(round(corners[NBR_COLUMNS*NBR_ROWS-1][0][1]))
        xalt.append(round(corners[(NBR_COLUMNS-1)*NBR_ROWS][0][0]))
        yalt.append(round(corners[(NBR_COLUMNS-1)*NBR_ROWS][0][1]))    
        for i in range(0, 4):
            sumV.append(xalt[i] + yalt[i])
        minV = min(sumV)
        maxV = max(sumV)
        logFileWrite(logFile,"Sum: "+str(sumV))
        for i in range(0, 4):
            if minV==sumV[i]:
                x1 = xalt[i]; y1 = yalt[i]
            if maxV==sumV[i]:
                x2 = xalt[i]; y2 = yalt[i]
        
        logFileWrite(logFile,"Endpoints: ("+str(x1)+","+str(y1)+") ; ("+str(x2)+","+str(y2)+")")
        # Draw and display the corners (ADD FRAMES)
        cv2.drawChessboardCorners(img, (NBR_COLUMNS,NBR_ROWS), corners,ret)

    else:
        logFileWrite(logFile,"Chessboard not found!")
        x1=-1; y1=-1; x2=-1; y2=-1
        
    cv2.imshow('img',img)
    cv2.waitKey(1)
        
    return x1,y1,x2,y2