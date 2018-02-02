#!/usr/bin/env python
#encoding=utf-8
"""
获取位置信息，通过网络请求发送数据
"""

import threading
import requests
import json
import uuid
import datetime
import re
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
    # test id
    return "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"

def getToken():
    url = "http://w3cloud-beta.huawei.com/umws4login/services/Authentication/wsdl/Authentication.wsdl"
    tmp_token = ""
    try:
        client = Client(url)
        tmp_token = client.service.getTokenByAppCredential({
            'applicationId': 'huawei.supply.mes.mesplus',
            'credential': 'L)j^Kp5vz1PMdDtT4)zL5(zLds5Gf>Ktmmo!yeH7Et+I+bjXwJM1469497455770',
            'userId': None,
            'ip': None,
            'attrsMap2Str': None,
        })
    except Exception as e:
        print(e)
	mtoken = "huawei.supply.mes.mesplus" + ":" + tmp_token
    return base64.b64encode(mtoken)

if __name__ == "__main__":
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
                    'timeout': str(5 * 60 * 1000),
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
                    "reqTime": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "wareHouseCode": "WMWHSE3",
                    "sourceName": "LINDE",
                },
                "INFO": {
                    "appid": "huawei.supply.mes.mesplus",
                    "rtype": "union",
                    "supappid": "ab8cc7ac3b81422594f2a48b5c841f97",
                    "traceid": datetime.datetime.now().strftime("%Y%m%d%H%M%S") +
                        str(uuid.uuid1()).replace("-", "")[:5].upper(),
                    "userid": "-default-",
                }
            }
            print(json.dumps(data, indent=4))
            try:
                requests.post(url, data=json.dumps(data), headers=headers)
            except:
                pass
        time.sleep(1)
