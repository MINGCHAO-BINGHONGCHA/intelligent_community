#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String
import os

# 你的虚拟环境路径
VENV_PYTHON = "/home/ming/tts-env/bin/python3"
PIPER_CLI = "/home/ming/tts-env/bin/piper"
MODEL_PATH = "/home/ming/tts_models/zh_CN-huayan-medium.onnx"
OUTPUT_WAV = "/tmp/tts_output.wav"

def tts_callback(msg):
    text = msg.data.strip()
    rospy.loginfo("TTS 收到文本: %s" % text)

    # 调用 Piper 生成 wav
    cmd = f'echo "{text}" | {PIPER_CLI} --model {MODEL_PATH} --output-file {OUTPUT_WAV}'
    os.system(cmd)

    # 播放 wav（使用 aplay 或 play）
    subprocess.call(["aplay", OUTPUT_WAV])

def tts_node():
    rospy.init_node("tts_node")
    rospy.Subscriber("/tts", String, tts_callback)
    rospy.loginfo("TTS 节点已启动，等待文本...")
    rospy.spin()

if __name__ == "__main__":
    tts_node()

