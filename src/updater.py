#!/usr/bin/env python
#encoding=utf-8
"""
获取位置信息，通过网络请求发送数据
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading
import requests
import json
import uuid
import datetime
import re
from tf.listener import TransformListener
import time
from zeep import Client
import base64

SERVER_IP = "127.0.0.1"
SERVER_PORT = "9900"
API_URL = "createHeartBeatReport"

LASTEST_POSE = [0, 0]
POSE_LOCK = threading.RLock()
SHARPLINK_LOG_FILE= "/home/xiaoqiang/Documents/ros/devel/lib/sharplink/server.log"

TRANS_POSE = None

def get_my_id():
    log_file = open(SHARPLINK_LOG_FILE)
    contents = log_file.read()
    log_file.close()
    mid_search = re.search(r"[0-9]+,\sID:\s(?P<id>[0-9A-F]{76})", contents)
    mid = mid_search.group("id")
    return mid

def odom_cb(odom):
    global POSE_LOCK, LASTEST_POSE
    pose_stamped = PoseStamped()
    pose_stamped.header = odom.header
    pose_stamped.pose = odom.pose.pose
    trans_pose_stamped = None
    try:
        TRANS_POSE.waitForTransform(pose_stamped.header.frame_id, "/map",
            pose_stamped.header.stamp, rospy.Duration(0.1))
        trans_pose_stamped = TRANS_POSE.transformPose('/map', pose_stamped)
    except (tf.LookupException,
        tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
        # rospy.logwarn("tf transform from /odom /map error")
        print(e)
    if trans_pose_stamped == None:
        return
    with POSE_LOCK:
        res = {}
        res["x"] = trans_pose_stamped.pose.position.x
        res["y"] = trans_pose_stamped.pose.position.y
        LASTEST_POSE = [int(1000 * res["x"]), int(1000 * res["y"]), ]

def getToken():
    url = "http://w3cloud-beta.huawei.com/umws4login/services/Authentication/wsdl/Authentication.wsdl"
    client = Client(url)
    tmp_token = client.service.getTokenByAppCredential({
        'applicationId': 'huawei.supply.mes.mesplus',
        'credential': 'L)j^Kp5vz1PMdDtT4)zL5(zLds5Gf>Ktmmo!yeH7Et+I+bjXwJM1469497455770',
        'userId': None,
        'ip': None,
        'attrsMap2Str': None,
    })
    mtoken = "huawei.supply.mes.mesplus" + ":" + tmp_token
    return base64.b64encode(mtoken)

if __name__ == "__main__":
    # rospy.init_node("pose_updater", anonymous=True)
    # rate = rospy.Rate(0.5)
    # TRANS_POSE = TransformListener()
    # rospy.Subscriber("/xqserial_server/Odom", Odometry, odom_cb)
    # while not rospy.is_shutdown():
    while True:
        with POSE_LOCK:
            headers = {
                'Content-Type': 'application/json',
                'Accept-Language': 'zh-cn, en; q=0.5',
                'Accept-Encoding': 'gzip, deflate',
                'User-Agent': 'Mozilla/4.0 (compatible; MSIE 8.0; Windows NT 5.1; Trident/4.0; aff-kingsoft-ciba; .NET4.0C; .NET4.0E; .NET CLR 2.0.50727; .NET CLR 3.0.04506.648; .NET CLR 3.5.21022)',
                'Authorization': 'Basic ' + getToken(),
                'HAEHead': json.dumps({
                    'version': '1.0',
                    'format': 'json',
                    'actiontype': 'R',
                    'region': 'szxgl',
                    'env': 'haeglsit10',
                    'timeout': '' + 5 * 60 * 1000,
                    'service': 'mesmlmservice',
                    'module': 'lwx372618',
                    'method': 'head',
                    'action': 'lwx372618.head',
                })
            }
            url = "http://{SERVER_IP}:{SERVER_PORT}/{API_URL}".format(
                SERVER_IP=SERVER_IP,
                API_URL=API_URL,
                SERVER_PORT=SERVER_PORT,
            )

            data = {
                "BO": {
                    "data": [{
                        "state": 0,
                        "deviceNo": get_my_id()[:30],
                        "currentX": LASTEST_POSE[0],
                        "currentY": LASTEST_POSE[1],
                        "currentSpeed": "0.0",
                        "buildingFloor": "1",
                        "batteryLevel": 100,
                        "currentPlanDistance": 0,
                        "currentFilishDistance": 0,
                        "cmdId": "",
                    }],
                    "reqCode": str(uuid.uuid1()).replace("-", "")[:30].upper(),
                    "reqTime": datetime.datetime.now().strftime("%G-%m-%e %H:%M:%S"),
                    "wareHouseCode": "WMWHSE3",
                    "sourceName": "LINDE",
                },
                "INFO": {
                    "appid": "huawei.supply.mes.mesplus",
                    "rtype": "union",
                    "supappid": "ab8cc7ac3b81422594f2a48b5c841f97",
                    "traceid": datetime.datetime.now().strftime("%G%m%e%H%M%S") +
                        str(uuid.uuid1()).replace("-", "")[:5].upper(),
                    "userid": "-default-",
                }
            }
            print(json.dumps(data, indent=4))
            try:
                requests.post(url, data=json.dumps(data), headers=headers)
            except:
                pass
        # rate.sleep()
        time.sleep(1)
