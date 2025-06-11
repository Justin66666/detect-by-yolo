# Recognizing Signs Autopilot

A traffic sign detection and recognition system using YOLO (You Only Look Once) deep learning model.

## Project Description

This project implements an automated traffic sign recognition system designed for autonomous driving applications. It uses YOLOv5 for real-time detection and classification of various traffic signs.

## Features

- Real-time traffic sign detection using YOLOv5
- Support for multiple sign types including:
  - U-turn signs
  - Straight/left turn signs
  - And more traffic signs
- Video processing capabilities
- High accuracy detection and classification

## Files Structure

- `UturnDetect.py` - U-turn sign detection implementation
- `StraightleftDetect.py` - Straight and left turn sign detection
- `research.py` - Research and experimental code
- `yolov5/` - YOLOv5 model implementation and utilities
- `Sample video.MOV` - Sample video for testing (excluded from git due to size)

## Requirements

See `yolov5/requirements.txt` for detailed dependencies.

## Usage

1. Install dependencies:
```bash
pip install -r yolov5/requirements.txt
```

2. Run detection on video:
```bash
python UturnDetect.py
# or
python StraightleftDetect.py
```

## Model

This project uses YOLOv5 pre-trained models for object detection, fine-tuned for traffic sign recognition.

## License

This project is open source and available under the MIT License.
