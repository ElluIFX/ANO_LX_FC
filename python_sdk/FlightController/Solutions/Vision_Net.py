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
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=4)
        label = f"QRcode:{conf:.2f}"
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

    def __post_process(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
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
        ret = [(centers[i], confidences[i]) for i in indices]
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
        return ret

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 置信度)
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
        return self.__post_process(frame, outs)


class FastestDet:
    def __init__(self, confThreshold=0.3, nmsThreshold=0.4, drawOutput=False):
        """
        FastestDet 目标检测网络
        confThreshold: 置信度阈值
        nmsThreshold: 非极大值抑制阈值
        """
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        path_names = os.path.join(path, "FastestDet.names")  # 识别类别
        path_onnx = os.path.join(path, "FastestDet.onnx")
        self.classes = list(map(lambda x: x.strip(), open(path_names, "r").readlines()))
        self.inpWidth = 512
        self.inpHeight = 512
        self.net = cv2.dnn.readNet(path_onnx)
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.H, self.W = 32, 32
        self.grid = self.__make_grid(self.W, self.H)
        self.drawOutput = drawOutput

    def __make_grid(self, nx=20, ny=20):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    def __postprocess(self, frame, outs):
        """
        后处理, 对输出进行筛选
        """
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        classIds = []
        confidences = []
        boxes = []
        centers = []
        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId] * detection[0]
            if confidence > self.confThreshold:
                center_x = int(detection[1] * frameWidth)
                center_y = int(detection[2] * frameHeight)
                width = int(detection[3] * frameWidth)
                height = int(detection[4] * frameHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])
                centers.append((center_x, center_y))
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confThreshold, self.nmsThreshold
        )
        indices = np.array(indices).flatten().tolist()
        ret = [(centers[i], self.classes[classIds[i]], confidences[i]) for i in indices]
        if self.drawOutput:
            for i in indices:
                box = boxes[i]
                left = box[0]
                top = box[1]
                width = box[2]
                height = box[3]
                self.__drawPred(
                    frame,
                    classIds[i],
                    confidences[i],
                    left,
                    top,
                    left + width,
                    top + height,
                )
        return ret

    def __drawPred(self, frame, classId, conf, left, top, right, bottom):
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
        label = f"{self.classes[classId]}: {conf:.2f}"
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv2.putText(
            frame,
            label,
            (left, top - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            thickness=1,
        )

    def __sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def detect(self, frame):
        """
        执行识别
        return: 识别结果列表: (中点坐标, 类型名称, 置信度)
        """
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.inpWidth, self.inpHeight))
        self.net.setInput(blob)
        pred = self.net.forward(self.net.getUnconnectedOutLayersNames())[0]
        pred[:, 3:5] = self.__sigmoid(pred[:, 3:5])  ###w,h
        pred[:, 1:3] = (np.tanh(pred[:, 1:3]) + self.grid) / np.tile(
            np.array([self.W, self.H]), (pred.shape[0], 1)
        )  ###cx,cy
        return self.__postprocess(frame, pred)
