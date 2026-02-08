#!/usr/bin/env python3
"""
RF-DETR Nano Detection CLI
===========================
Command-line interface for object detection with custom or COCO model.
Need venv containing all modules to run, best if GPU accelerated. Use `conda activate ai` or your own solution.

Usage:
    # COCO model (default)
    python detect.py --image dog.jpg
    
    # Custom model
    python detect.py --image dog.jpg --model weights/best.pth --classes class_names.txt
    
    # With confidence threshold
    python detect.py --image dog.jpg --model weights/best.pth --classes class_names.txt --confidence 0.3
    
    # Skip visualization
    python detect.py --image dog.jpg --no-viz
"""
import argparse
import json
from pathlib import Path
try:
    from .rf_detr_nano import NanoDetector
except ImportError:
    from rf_detr_nano import NanoDetector # Works when running as standalone script


def load_class_names(filepath):
    """Load class names from text file (one class per line)."""
    with open(filepath, 'r') as f:
        return [line.strip() for line in f if line.strip()]


def save_json(detections, filepath):
    """Save detections to JSON file."""
    with open(filepath, 'w') as f:
        json.dump(detections, f, indent=2)
    print(f"Saved JSON: {filepath}")


def print_detections(detections):
    """Print detection summary to console."""
    print(f"\n{'='*60}")
    print(f"DETECTIONS: {len(detections)}")
    print(f"{'='*60}")
    
    if not detections:
        print("No objects detected")
        return
    
    for det in detections:
        print(f"\n[{det['detection_id']}] {det['class_name'].upper()}")
        print(f"    Confidence:  {det['confidence']:.4f}")
        print(f"    BBox:        ({det['bbox']['x_min']:.1f}, {det['bbox']['y_min']:.1f}) -> "
              f"({det['bbox']['x_max']:.1f}, {det['bbox']['y_max']:.1f})")
        print(f"    Center:      ({det['center']['x']}, {det['center']['y']})")


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(
        description='RF-DETR Nano object detection with custom or COCO model',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Use default COCO model
    python detect.py --image dog.jpg
    
    # Use custom model
    python detect.py --image my_image.jpg --model weights/best.pth --classes classes.txt
    
    # Adjust confidence
    python detect.py --image dog.jpg --confidence 0.3
    
    # Skip visualization
    python detect.py --image dog.jpg --no-viz
        """
    )
    parser.add_argument(
        '--image', '-i',
        required=True,
        help='Path to input image'
    )
    parser.add_argument(
        '--model', '-m',
        default=None,
        help='Path to custom model weights (.pth file). If not provided, uses COCO weights'
    )
    parser.add_argument(
        '--classes', '-cl',
        default=None,
        help='Path to class names file (one class per line). Required for custom models'
    )
    parser.add_argument(
        '--confidence', '-c',
        type=float,
        default=0.5,
        help='Confidence threshold (default: 0.5)'
    )
    parser.add_argument(
        '--output', '-o',
        default='detections',
        help='Output filename prefix (default: detections)'
    )
    parser.add_argument(
        '--no-viz',
        action='store_true',
        help='Skip creating annotated image'
    )
    
    args = parser.parse_args()
    
    # Validate image exists
    if not Path(args.image).exists():
        print(f"Error: Image not found: {args.image}")
        return 1
    
    # Validate custom model setup
    if args.model:
        if not Path(args.model).exists():
            print(f"Error: Model weights not found: {args.model}")
            return 1
        if not args.classes:
            print("Error: --classes is required when using a custom model")
            return 1
        if not Path(args.classes).exists():
            print(f"Error: Class names file not found: {args.classes}")
            return 1
    
    # Load class names if custom model
    class_names = None
    if args.classes:
        class_names = load_class_names(args.classes)
        print(f"Loaded {len(class_names)} classes: {class_names}")
    
    # Initialize detector
    detector = NanoDetector(
        confidence_threshold=args.confidence,
        model_path=args.model,
        class_names=class_names
    )
    
    # Load image
    print(f"\nLoading image: {args.image}")
    image = detector.load_image(args.image)
    print(f"Image size: {image.size[0]}x{image.size[1]}")
    
    # Run detection
    print(f"\nRunning detection (threshold: {args.confidence})...")
    detections, raw_detections = detector.detect(image)
    print(f"Found {len(detections)} detections")
    
    # Print results
    print_detections(detections)
    
    # Save outputs
    print(f"\n{'='*60}")
    print("SAVING OUTPUTS")
    print(f"{'='*60}\n")
    
    # Create output directories if they don't exist
    Path("results_json").mkdir(exist_ok=True)
    Path("results_annotated").mkdir(exist_ok=True)
    
    save_json(detections, f"results_json/{args.output}.json")    
    
    # Create visualization
    if not args.no_viz:
        viz_path = f"results_annotated/{args.output}_annotated.jpg"
        detector.visualize(image, raw_detections, output_path=viz_path)
    
    print(f"\n{'='*60}")
    print("COMPLETE!")
    print(f"{'='*60}\n")
    
    return 0


if __name__ == "__main__":
    exit(main())
