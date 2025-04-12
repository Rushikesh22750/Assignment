#!/usr/bin/env python3


import rospy
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import Image
from swaayatt_rushikesh.msg import DetectedObjects, DetectedObject


class YoloDetector:

    def __init__(self):

        rospy.init_node('yolo_detector', anonymous=True)

        self.input_size = (640, 640)  

        self.conf_threshold  =  0.4       
        self.nms_threshold  =  0.5         

        #### Here we are loading class names from coco

        names_path = "/home/rushi/swaayatt_ws/src/swaayatt_rushikesh/yolo_models/coco.names"
        with open(names_path, 'r') as f:
            self.class_names = [line.strip() for line in f.readlines()]
        


        #### Loading the yolov5 small

        self.model_path = "/home/rushi/swaayatt_ws/src/swaayatt_rushikesh/yolo_models/yolov5s.onnx"

        self.session = ort.InferenceSession(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
       
    

        # Creating publisher and subscriber

        self.sub = rospy.Subscriber("/processed_image", Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher("/object_detection", DetectedObjects, queue_size=1)

        rospy.loginfo("YoloDetector is ready and subscribed to /processed_image.")



    def preprocess_image(self, image_data, width, height):
        
        #### Shape is changed from...(height, width, 3) to (1, 3, 640, 640) which is required for yolo

        resized = np.resize(image_data, (self.input_size[1], self.input_size[0], 3))
        
        normalized = resized.astype(np.float32) / 255.0
      
        transposed = np.transpose(normalized, (2, 0, 1))
     
        batched = np.expand_dims(transposed, axis=0)
        return batched

        

    def image_callback(self, msg):

        # Converting ROS message to numpy array
        image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        rospy.loginfo("Image height: %d, Image width: %d", msg.height, msg.width)
        #image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        #image_data = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

    
        input_image = self.preprocess_image(image_data, msg.width, msg.height)
       
        detections = self.run_inference(input_image) ## Running Inference

        detections = self.apply_nms(detections)

        # Create a DetectedObjects message and add all final detections

        det_msg = DetectedObjects()
        det_msg.header = msg.header

        for det in detections:

            obj = DetectedObject()
            obj.class_name = self.class_names[det["class_id"]]
            obj.x, obj.y, obj.width, obj.height = det["bbox"]
            obj.confidence = det["confidence"]
            det_msg.objects.append(obj)

        self.pub.publish(det_msg)
        rospy.loginfo("Published %d detections", len(det_msg.objects))




    def run_inference(self, image):
    

        ##### Runs the yolo model. The expected output shape is (1, num_boxes, 85)
        ###   85 = 80(classes) + 4 (Bounding boxes size) + 1 (confidence)

        ### This will give bounding boxes without the Non max supression.

        outputs = self.session.run([self.output_name], {self.input_name: image})[0]
        print("Output shape:", outputs.shape)
        
        detections = []

        # Looping through each predicted box

        for i in range(outputs.shape[1]):
            row = outputs[0][i]
            conf = row[4]
            if isinstance(conf, np.ndarray):
                conf = np.max(conf)
            if conf >= self.conf_threshold:
                class_scores = row[5:]
                class_id = np.argmax(class_scores)
                class_score = class_scores[class_id]
                if class_score > 0.4:
                    cx, cy, w, h = row[0], row[1], row[2], row[3]

                    # Converting center format to (x, y, width, height)

                    bbox = [cx - w/2, cy - h/2, w, h]
                    detections.append({"class_id": class_id, "confidence": conf, "bbox": bbox})

        return detections



#####  Applying Non Mx Supression

    def apply_nms(self, detections):
       
        if len(detections) == 0:
            return []
        
        boxes = []
        scores = []

        for det in detections:
            x, y, w, h = det["bbox"]
            boxes.append([x, y, x + w, y + h])  ### (x1, y1, x2, y2)
            scores.append(det["confidence"])

        boxes = np.array(boxes)
        scores = np.array(scores)
        
        ### Sorting by confidence
        indices = scores.argsort()[::-1]
        keep = []

        while indices.size > 0:
            i = indices[0]
            keep.append(i)

            if indices.size == 1:
                break

            current_box = boxes[i]
            other_boxes = boxes[indices[1:]]

            # Calculating the intersection area

            xx1 = np.maximum(current_box[0], other_boxes[:, 0])
            yy1 = np.maximum(current_box[1], other_boxes[:, 1])
            xx2 = np.minimum(current_box[2], other_boxes[:, 2])
            yy2 = np.minimum(current_box[3], other_boxes[:, 3])

            inter_w = np.maximum(0, xx2 - xx1)
            inter_h = np.maximum(0, yy2 - yy1)
            inter_area = inter_w * inter_h
            
            # Calculating the union area
            area_current = (current_box[2] - current_box[0]) * (current_box[3] - current_box[1])
            area_other = (other_boxes[:, 2] - other_boxes[:, 0]) * (other_boxes[:, 3] - other_boxes[:, 1])
            union_area = area_current + area_other - inter_area
            
            # Calculating Intersection over Union (IOU)
            iou = inter_area / union_area
            
            remaining = np.where(iou <= self.nms_threshold)[0]
            indices = indices[remaining + 1]
        
        # Final detections

        final_detections = [detections[i] for i in keep]
        return final_detections

    def spin(self):
        rospy.spin()

if __name__ == "__main__":

    node = YoloDetector()

    node.spin()
