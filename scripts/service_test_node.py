import rospy
from pyqt_ros_example.srv import *

test_num = [0,0,0,0]
def callbackFunction(req):
    print("selected mode:",req.type)
    res = drawing_typeResponse()
    if test_num[0]%2==0:
        res.set_success = True
    else :
        res.set_success = False
    test_num[0] += 1
    return res
def callbackFunction2(req):
    print("set img time:",req.set_image_flag_time)
    res = set_img_flagResponse()
    if test_num[1]%2==0:
        res.set_success = True
    else :
        res.set_success = False
    test_num[1] += 1
    return res

def callbackFunction3(req):
    print("start calculate flag:",req.start_flag)
    res = start_calculate_flagResponse()
    if test_num[2]%2==0:
        res.set_success = True
    else :
        res.set_success = False
    test_num[2] += 1
    return res

def callbackFunction4(req):
    print("start drawing flag::",req.start_flag)
    res = start_drawing_flagResponse()
    if test_num[3]%2==0:
        res.set_success = True
    else :
        res.set_success = False
    test_num[3] += 1
    return res

def service_server():
    rospy.init_node('test_serveice_server')
    s1 = rospy.Service('/drawing_type', drawing_type, callbackFunction)
    s2 = rospy.Service('/set_img_flag', set_img_flag, callbackFunction2)
    s3 = rospy.Service('/calcuate_flag',start_calculate_flag, callbackFunction3)
    s4 = rospy.Service('/drawing_start_flag',start_drawing_flag, callbackFunction4)

    rospy.spin()

if __name__ == "__main__":
    service_server()