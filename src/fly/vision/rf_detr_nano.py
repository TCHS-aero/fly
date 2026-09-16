"""
Core logic for RF-DETR Nano Detection Module
Modified to support custom-trained models
"""

import time

import numpy as np
import supervision as sv
from PIL import Image
from rfdetr import RFDETRNano


class NanoDetector:
    """RF-DETR Nano object detector with custom or COCO weights.

    Class names are never supplied by the caller. A fine-tuned
    .pth carries whatever label set it was trained with.
    """

    def __init__(self, confidence_threshold=0.5, model_path=None):
        """
        Initialize detector.

        Args:
            confidence_threshold: Minimum confidence for detections (default: 0.5)
            model_path: Path to custom model weights (.pth file). If None, uses COCO weights
        """
        self.confidence_threshold = confidence_threshold
        self.model_path = model_path
        self.model = None
        self.class_names = None  # set by load_model()

    def load_model(self):
        """Load model with custom or COCO weights."""
        if self.model_path:
            print(f"Loading RF-DETR Nano model with custom weights from: {self.model_path}")
            self.model = RFDETRNano(weights=self.model_path)
        else:
            print("Loading RF-DETR Nano model with COCO weights...")
            self.model = RFDETRNano()
        # Kept here for logging only, detect() and visualize()
        # read the name for each detection off the detections themselves rather than
        # re-deriving it from this list (see the comment in detect()).
        self.class_names = self.model.class_names
        print(f"Model loaded! ({len(self.class_names)} classes)")

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
        # Start timer
        start_time = time.perf_counter()
        # Run inference
        detections = self.model.predict(image, threshold=self.confidence_threshold)
        # End timer
        end_time = time.perf_counter()
        inference_time = end_time - start_time

        # Extract information
        results = []
        for i in range(len(detections)):
            # Bounding box
            bbox = detections.xyxy[i].tolist()
            x_min, y_min, x_max, y_max = bbox

            # Indexing self.class_names[class_id] directly would
            # get this wrong for stock weights, since COCO category ids aren't
            # contiguous list positions.
            class_id = int(detections.class_id[i])
            class_name = str(detections.data["class_name"][i])

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

        return results, detections, inference_time

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
            class_name = str(detections.data["class_name"][i])
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
