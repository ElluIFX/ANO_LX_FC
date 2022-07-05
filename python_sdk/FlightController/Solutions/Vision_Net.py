import os
from array import array

import cv2
import numpy as np


class QRScanner_Yolo(object):
    def __init__(self, confThreshold=0.6, nmsThreshold=0.5, drawOutput=False):
        """
        YoloV3 二维码识别
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        drawOutput: 是否在图像上画出识别结果
        """
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.inpWidth = 416
        self.inpHeight = 416
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        cfg_path = os.path.join(path, "qrcode-yolov3-tiny.cfg")
        weights_path = os.path.join(path, "qrcode-yolov3-tiny.weights")
        self.net = cv2.dnn.readNet(cfg_path, weights_path)
        self.drawOutput = drawOutput

    def __drawPred(self, frame, conf, left, top, right, bottom):
        # return # 不用画框的话就取消这个注释
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=4)
        label = f"QRCODE:{conf:.2f}"
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv2.putText(
            frame,
            label,
            (left, top - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            thickness=2,
        )
        return frame

    def __post_process(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        classIds = []
        confidences = []
        boxes = []
        centers = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    boxes.append([left, top, width, height])
                    confidences.append(float(confidence))
                    centers.append((center_x, center_y))
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        centers_indices = [centers[i] for i in indices]
        confidences_indices = [confidences[i] for i in indices]
        if self.drawOutput:
            for i in indices:
                box = boxes[i]
                left = box[0]
                top = box[1]
                width = box[2]
                height = box[3]
                self.__drawPred(
                    frame,
                    confidences[i],
                    left,
                    top,
                    left + width,
                    top + height,
                )
        return centers_indices, confidences_indices

    def detect(self, frame):
        """
        执行识别
        return: 识别结果中点坐标列表, 置信度列表
        """
        blob = cv2.dnn.blobFromImage(
            frame,
            1 / 255.0,
            (self.inpWidth, self.inpHeight),
            [0, 0, 0],
            swapRB=True,
            crop=False,
        )
        # 加载网络
        self.net.setInput(blob)
        # 前向传播
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        centers, confidences = self.__post_process(frame, outs)
        return centers, confidences
