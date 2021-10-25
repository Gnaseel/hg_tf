
#!/usr/bin/env python
import rospy
import tf
from TF_converter import TFcvt
from common import TFcfg
import numpy as np


TF_cfg = TFcfg()
TF_cvt = TFcvt()
def getRobotTransform():
    #  TF between robot base and hand
    try:
        (trans, rot) = TF_cfg.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None, None
    return trans, rot


def getCamTransform():
    # TF between camera and chessboard
    try:
        (trans, rot) = TF_cfg.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None, None
    return trans, rot
def getReverse(list):
    li2=[]
    for i in list:
        li2.append(i*-1)
    return li2
def main():
    rospy.init_node("HandEye Evaluatyor")

    he_rot = [0, 16.25, 0]

    tRc = TF_cvt.euler_to_rotMat(*he_rot)
    re_he_rot = getReverse(he_rot)
    cRt = TF_cvt.euler_to_rotMat(*re_he_rot)

    bRt = np.zeros((4,4))
    tRb = np.zeros((4,4))
    cRw = np.zeros((4,4))
    bRw = np.zeros((4,4))
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

        robot_trans, robot_rot = getRobotTransform()
        if robot_trans is not None and robot_rot is not None:
            bRt = TF_cvt.euler_to_rotMat(*robot_rot)
            re_robot_rot = getReverse(robot_rot)
            tRb = TF_cvt.euler_to_rotMat(*re_robot_rot)
            # ~~~~
            continue
        cam_trans, cam_rot = getCamTransform()
        if cam_trans is not None and cam_rot is not None:
            # ~~~~
            cRw = TF_cvt.euler_to_rotMat(*cam_rot)
            re_cam_rot = getReverse(cam_rot)
            wRc = TF_cvt.euler_to_rotMat(*re_cam_rot)
            continue

        bRw = np.dot(np.dot(bRt,tRc), cRw)

        delR1 = np.dot(wRc, cRt)
        delR2 = np.dot(tRb, bRw)
        print("Del1 = {}".format(delR1))
        print("Del2 = {}".format(delR2))

        tTc = np.array( [ [TF_cfg.h_e_array[0][3]], [TF_cfg.h_e_array[1][3]], [TF_cfg.h_e_array[2][3]], [0] ])
        tTb = -1 * np.array([ [robot_trans[0]], [robot_trans[1]], [robot_trans[2]], [0] ])
        cTw = np.array([ [cam_trans[0]], [cam_trans[1]], [cam_trans[2]], [0] ])
        
        totalT = np.dot(bRt * np.dot(TF_cfg.h_e_array*cTw))
        bTw = np.array([ [totalT[0][3] + tTb[0]*-1], [totalT[1][3] + tTb[1]*-1], [totalT[2][3] + tTb[2]*-1] ])
        
        delT = np.dot(tRb, bTw) + tTb - np.dot(tRc, cTw) + tTc

        print("Del T = {}".format(delT))


    return
if __name__=="__main__":
   
    main()