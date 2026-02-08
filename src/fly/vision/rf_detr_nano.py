"""
Core logic for RF-DETR Nano Detection Module
Modified to support custom-trained models
"""

from PIL import Image
import numpy as np
from rfdetr import RFDETRNano
from rfdetr.util.coco_classes import COCO_CLASSES
import supervision as sv


class NanoDetector:
    """RF-DETR Nano object detector with custom or COCO weights."""
    
    def __init__(self, confidence_threshold=0.5, model_path=None, class_names=None):
        """
        Initialize detector.
        
        Args:
            confidence_threshold: Minimum confidence for detections (default: 0.5)
            model_path: Path to custom model weights (.pth file). If None, uses COCO weights
            class_names: List of class names for custom model. If None, uses COCO_CLASSES
        """
        self.confidence_threshold = confidence_threshold
        self.model_path = model_path
        self.class_names = class_names if class_names is not None else COCO_CLASSES
        self.model = None
        
    def load_model(self):
        """Load model with custom or COCO weights."""
        if self.model_path:
            print(f"Loading RF-DETR Nano model with custom weights from: {self.model_path}")
            self.model = RFDETRNano(weights=self.model_path)
        else:
            print("Loading RF-DETR Nano model with COCO weights...")
            self.model = RFDETRNano()
        print("Model loaded!")
        
    def load_image(self, image_path):
        """
        Load image from file.
        
        Args:
            image_path: Path to image file
            
        Returns:
            PIL Image in RGB format
        """
        image = Image.open(image_path)
        if image.mode != 'RGB':
            image = image.convert('RGB')
        return image
    
    def detect(self, image):
        """
        Run object detection on image.
        
        Args:
            image: PIL Image
            
        Returns:
            tuple: (detections_dict, raw_detections)
                - detections_dict: List of detection dictionaries
                - raw_detections: Raw supervision Detections object
        """
        if self.model is None:
            self.load_model()
        
        # Run inference
        detections = self.model.predict(image, threshold=self.confidence_threshold)
        
        # Extract information
        results = []
        for i in range(len(detections)):
            # Bounding box
            bbox = detections.xyxy[i].tolist()
            x_min, y_min, x_max, y_max = bbox
            
            # Class
            class_id = int(detections.class_id[i])
            class_name = self.class_names[class_id]
            
            # Confidence
            confidence = float(detections.confidence[i])
            
            # Center pixel
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)
            
            result = {
                "detection_id": i,
                "class_name": class_name,
                "class_id": class_id,
                "confidence": round(confidence, 4),
                "bbox": {
                    "x_min": round(x_min, 2),
                    "y_min": round(y_min, 2),
                    "x_max": round(x_max, 2),
                    "y_max": round(y_max, 2)
                },
                "center": {
                    "x": center_x,
                    "y": center_y
                }
            }
            results.append(result)
        
        return results, detections
    
    def visualize(self, image, detections, output_path=None):
        """
        Create annotated image with bounding boxes and labels.
        
        Args:
            image: Original PIL Image
            detections: Raw detections from detect()
            output_path: Optional path to save annotated image
            
        Returns:
            PIL Image with annotations
        """
        # Convert PIL to numpy for supervision
        image_np = np.array(image)
        
        # Create labels with class name and confidence
        labels = []
        for i in range(len(detections)):
            class_id = int(detections.class_id[i])
            class_name = self.class_names[class_id]
            confidence = float(detections.confidence[i])
            labels.append(f"{class_name} {confidence:.3f}")
        
        # Annotate with bounding boxes
        box_annotator = sv.BoxAnnotator()
        annotated = box_annotator.annotate(scene=image_np.copy(), detections=detections)
        
        # Annotate with labels
        label_annotator = sv.LabelAnnotator()
        annotated = label_annotator.annotate(scene=annotated, detections=detections, labels=labels)
        
        # Convert back to PIL
        annotated_image = Image.fromarray(annotated)
        
        # Save if path provided
        if output_path:
            annotated_image.save(output_path)
            print(f"Saved annotated image: {output_path}")
        
        return annotated_image
