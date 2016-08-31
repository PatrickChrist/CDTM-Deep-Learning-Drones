import cv2
import cv2.aruco as aruco
import numpy as np

print cv2.__version__
 
cap = cv2.VideoCapture(0)
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
 
    #print(parameters)
 
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    center = (0, 0)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if corners:
        for corner in corners[0]:
            dim = corner.shape
            print corner
            s = np.sum(corner, axis=0)
            print "sum", s
            s = s / dim[0]
            center = (s[0], s[1])

 
    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs
 
    gray = aruco.drawDetectedMarkers(gray, corners)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.circle(color, center, 2, (0, 0, 255), 2)
    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame', color)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
