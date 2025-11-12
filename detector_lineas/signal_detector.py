#!/usr/bin/env python3
"""
Signal Detector Module using YOLO

Handles traffic signal detection using a trained YOLO model.
Follows Single Responsibility Principle - only handles signal detection.
"""

import cv2
import numpy as np
from typing import Optional, List, Dict, Tuple
import threading


class SignalDetector:
    """
    Detector for traffic signals using YOLO model.
    
    Handles:
    - Loading YOLO model
    - Processing frames for signal detection
    - Returning detected signals with confidence
    """
    
    def __init__(self, model_path: str, conf_threshold: float = 0.5):
        """
        Initialize signal detector.
        
        Args:
            model_path: Path to YOLO model file (.pt)
            conf_threshold: Confidence threshold for detections (default: 0.5)
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None
        self.device = 'cpu'
        self._lock = threading.Lock()
        self._load_model()
    
    def _get_device(self):
        """
        Detect and return the best available device (GPU/CPU).
        Prioritizes GPU for Jetson devices.
        """
        try:
            import torch
            if torch.cuda.is_available():
                device = 'cuda'
                device_name = torch.cuda.get_device_name(0) if torch.cuda.device_count() > 0 else 'CUDA'
                return device, device_name
            else:
                return 'cpu', 'CPU'
        except ImportError:
            # If torch not available, try to detect via ultralytics
            try:
                from ultralytics.utils import torch_utils
                device = torch_utils.select_device('')
                device_name = 'GPU' if 'cuda' in str(device) else 'CPU'
                return str(device), device_name
            except:
                return 'cpu', 'CPU'
    
    def _load_model(self):
        """Load YOLO model with GPU support if available."""
        try:
            # Detect device
            device, device_name = self._get_device()
            
            # Try ultralytics (YOLOv8)
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                # Ultralytics automatically uses GPU if available, but we store device info
                self.model_type = 'ultralytics'
                self.device = device
                print(f"✓ Signal detector loaded (Ultralytics YOLO): {self.model_path}")
                print(f"  Device: {device_name} ({device})")
                if device == 'cuda':
                    print(f"  GPU acceleration: ENABLED")
                else:
                    print(f"  GPU acceleration: DISABLED (using CPU)")
            except ImportError:
                # Try torch with YOLOv5
                try:
                    import torch
                    self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
                    # Move model to GPU if available
                    if device == 'cuda':
                        self.model = self.model.to(device)
                    self.model_type = 'yolov5'
                    self.device = device
                    print(f"✓ Signal detector loaded (YOLOv5): {self.model_path}")
                    print(f"  Device: {device_name} ({device})")
                except Exception as e:
                    raise ImportError(f"Could not load YOLO model. Install ultralytics: pip install ultralytics. Error: {e}")
        except Exception as e:
            print(f"⚠ Warning: Could not load signal detector: {e}")
            print("   Signal detection will be disabled.")
            self.model = None
            self.device = 'cpu'
    
    def is_available(self) -> bool:
        """Check if model is loaded and available."""
        return self.model is not None
    
    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        Detect signals in a frame.
        
        Args:
            frame: Input frame (BGR format)
        
        Returns:
            List of detections, each with:
            - 'class': class name or id
            - 'confidence': confidence score
            - 'bbox': [x1, y1, x2, y2] bounding box coordinates
            - 'center': (x, y) center point
        """
        if not self.is_available():
            return []
        
        detections = []
        
        try:
            with self._lock:
                if self.model_type == 'ultralytics':
                    # YOLOv8 (ultralytics) - automatically uses GPU if available
                    # Specify device explicitly for Jetson
                    results = self.model(frame, conf=self.conf_threshold, verbose=False, device=self.device)
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            cls = int(box.cls[0].cpu().numpy())
                            class_name = result.names[cls] if hasattr(result, 'names') else str(cls)
                            
                            detections.append({
                                'class': class_name,
                                'class_id': cls,
                                'confidence': conf,
                                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                            })
                
                elif self.model_type == 'yolov5':
                    # YOLOv5
                    results = self.model(frame)
                    for *box, conf, cls in results.xyxy[0].cpu().numpy():
                        x1, y1, x2, y2 = map(int, box)
                        detections.append({
                            'class': self.model.names[int(cls)] if hasattr(self.model, 'names') else str(int(cls)),
                            'class_id': int(cls),
                            'confidence': float(conf),
                            'bbox': [x1, y1, x2, y2],
                            'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                        })
        
        except Exception as e:
            print(f"Error in signal detection: {e}")
            return []
        
        return detections
    
    def draw_detections(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        Draw detections on frame.
        
        Args:
            frame: Input frame (BGR format)
            detections: List of detections from detect()
        
        Returns:
            Frame with detections drawn
        """
        output_frame = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            confidence = det['confidence']
            
            # Draw bounding box
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(output_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), (0, 255, 0), -1)
            cv2.putText(output_frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return output_frame
    
    def get_most_confident(self, detections: List[Dict]) -> Optional[Dict]:
        """
        Get the most confident detection.
        
        Args:
            detections: List of detections
        
        Returns:
            Most confident detection or None
        """
        if not detections:
            return None
        
        return max(detections, key=lambda x: x['confidence'])

