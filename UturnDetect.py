# YOLOv5 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (macOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
import platform
import sys
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode, time_sync

import collections
import serial
import time



import rospy #引入 rospy 库，ROS 的 Python 接口，用于节点的创建和消息的发布/订阅
from demo_py.msg import DetectionResults


traffic_light_deque = collections.deque(maxlen=15)
dynamic_sign_deque = collections.deque(maxlen=15)
static_sign_deque = collections.deque(maxlen=15)

traffic_light_deque.append('None')
dynamic_sign_deque.append('None')
static_sign_deque.append('None')

traffic_light_list = ['red', 'green', 'yellow']
dynamic_sign_list = ['straight', 'straightleft', 'straightright', 'left', 'right', 'uturn']
static_sign_list = ['p', 'crosswalk', 'stopline', 'yellowsoildline', 'changeline']

traffic_light_condition = False #True represent Green, False represent Red and Yellow
adjust_flag = False #True when adjust

traffic_light_most_common = ''
dynamic_sign_most_common = ''
static_sign_most_common = ''

current_state = 'straight'

stopline_center = 0
yellowsoildline_center = 200
uturn_center = 200

portx = '/dev/ttyUSB0'
bps = 115200
timex = None
ser = serial.Serial(portx,bps,timeout=timex)

def stop_process():
    global stopline_center
    global current_state
    start_time = time.time()
    global ser
    while True:
        print('===IN THE STOP LOOP===')
        end_time = time.time()
        if end_time - start_time >= 0.8:
            ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')
            current_state = 'stop'
            stopline_center = 0
            return
            
def adjust_left_process():
    global yellowsoildline_center
    global ser
    adjust_flag = True
    start_time = time.time()
    ser.write(b'\xFF\xFC\x07\x10\xF7\x20\x10\x20\x5E')  #左转
    while True:
        print('===IN THE ADJUST LEFT LOOP===')
        end_time = time.time()
        if end_time - start_time >= 0.3:
            ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99') #直行
            yellowsoildline_center = 200
            adjust_flag = False
            return
            
def adjust_right_process():
    global yellowsoildline_center
    global ser
    adjust_flag = True
    start_time = time.time()
    ser.write(b'\xFF\xFC\x07\x10\x20\x20\xDF\x20\x56') #右转
    while True:
        print('===IN THE ADJUST RIGHT LOOP===')
        end_time = time.time()
        if end_time - start_time >= 0.3:
            ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99') #直行
            yellowsoildline_center = 200
            adjust_flag = False
            return
            
def uturn_process():
    global adjust_flag
    global uturn_center
    global ser
    adjust_flag = True #停止进行黄实线校准检验
    start_time = time.time()
    while True:
        print('===IN THE UTURN FIERST STRAIGHT PROCESS===')
        end_time = time.time()
        if end_time - start_time >= 2: #直行两秒
            ser.write(b'\xFF\xFC\x07\x10\xDF\xDF\x20\x20\x15')#逆时针旋转1.3秒
            break
    start_time = time.time()
    while True:
        print('===IN THE UTURN FIRST TURN LEFT PROCESS===')
        end_time = time.time()
        if end_time - start_time >= 1.3:
            ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99')#直行1.3秒
            break
            break
    start_time = time.time()
    while True:
        print('===IN THE UTURN SECOND STRAIGHT PROCESS===')
        end_time = time.time()
        if end_time - start_time >= 1.3:
            ser.write(b'\xFF\xFC\x07\x10\xDF\xDF\x20\x20\x15')#逆时针旋转1.3秒
            break
    start_time = time.time()
    while True:
        print('===IN THE UTURN SECOND TURN LEFT PROCESS===')
        end_time = time.time()
        if end_time - start_time >= 1.3:
            ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99') #恢复直行
            uturn_center = 0
            adjust_flag = False #恢复进行 黄实线校准检验
            return

def detector():
    # 初始化ROS节点
    rospy.init_node('object_detector')
    # 创建一个Publisher对象，允许发布消息到一个特定的话题,这里的参数包括话题名称detection_results、消息类型String和队列大小queue_size
    pub = rospy.Publisher('detection_results', String, queue_size=10) 
    rate = rospy.Rate(10)  # 设置频率为 10Hz

    while not rospy.is_shutdown(): # rospy.is_shutdown() 检查是否有外部请求（如用户或系统）关闭了 ROS 节点
        # 这里调用目标检测代码
        msg = DetectionResults()
        # 我们可以根据每一帧所需要的信息给msg的不同参数进行定义，例子如下
        msg.stopline_center = stopline_center
        msg.current_state = current_state
        
        rospy.loginfo(f"Publishing: {msg}")  # 在 ROS 日志中打印信息
        pub.publish(msg) # 发布消息
        rate.sleep()# 等待，以保持设定的发布频率

@smart_inference_mode()
def run(
    weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
    source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
    data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
    imgsz=(640, 640),  # inference size (height, width)
    conf_thres=0.25,  # confidence threshold
    iou_thres=0.45,  # NMS IOU threshold
    max_det=1000,  # maximum detections per image
    device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    view_img=False,  # show results
    save_txt=False,  # save results to *.txt
    save_conf=False,  # save confidences in --save-txt labels
    save_crop=False,  # save cropped prediction boxes
    nosave=False,  # do not save images/videos
    classes=None,  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False,  # class-agnostic NMS
    augment=False,  # augmented inference
    visualize=False,  # visualize features
    update=False,  # update all models
    project=ROOT / 'runs/detect',  # save results to project/name
    name='exp',  # save results to project/name
    exist_ok=False,  # existing project/name ok, do not increment
    line_thickness=3,  # bounding box thickness (pixels)
    hide_labels=False,  # hide labels
    hide_conf=False,  # hide confidences
    half=False,  # use FP16 half-precision inference
    dnn=False,  # use OpenCV DNN for ONNX inference
):
  
    global traffic_light_deque
    global dynamic_sign_deque
    global static_sign_deque
    
    global traffic_light_list
    global dynamic_sign_list
    global static_sign_list
    
    global traffic_light_condition
    
    global traffic_light_most_common
    
    global stopline_flag
    global adjust_flag
    
    global stopline_center
    global yellowsoildline_center
    global uturn_center
    
    global ser
    
    global current_state
    
    turn_left_straight_y_center = 0
    turn_left_straight_x_center = 0
    
    turn_left_very_special_flag = True

    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1  # batch_size
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], [0.0, 0.0, 0.0]
    
    ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99')
    current_state = 'straight'
    
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred = model(im, augment=augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(f'{txt_path}.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        name = names[c]
                        
                        if name in traffic_light_list:
                            traffic_light_deque.append(name)
                        elif name in dynamic_sign_list:
                            dynamic_sign_deque.append(name)                     
                        elif name in static_sign_list:
                            static_sign_deque.append(name)
                        
                        if name == 'crosswalk' and  (int(xyxy[1]) + int(xyxy[3])) / 2 > 360: #Since stopline is not sensable in our model, we use crosswalk to represent stopline
                            stopline_center = (int(xyxy[1]) + int(xyxy[3])) / 2
                        
                        if name == 'uturn' and  (int(xyxy[1]) + int(xyxy[3])) / 2 > 360: # 当uturn在一定范围内，更新uturn center
                            uturn_center = (int(xyxy[1]) + int(xyxy[3])) / 2
 
                        if name == 'yellowsoildline' and not adjust_flag: 
                            yellowsoildline_center = (int(xyxy[0]) + int(xyxy[2])) / 2
                        
                    if save_crop:
                        save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
                      
            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)
            
            traffic_light_most_common = collections.Counter(traffic_light_deque).most_common(1)[0][0]
            dynamic_sign_most_common = collections.Counter(dynamic_sign_deque).most_common(1)[0][0]
            static_sign_most_common = collections.Counter(static_sign_deque).most_common(1)[0][0]
            
            print('Traffic Light Most Common:', traffic_light_most_common)
            print('Dynamic Sign Most Common:', dynamic_sign_most_common)
            print('Static Sign Most Common:', static_sign_most_common)
            
            print('Dynamic Sign Deque:', dynamic_sign_deque)
            
            if traffic_light_most_common == 'red' or traffic_light_most_common == 'yellow':
                traffic_light_condition = True
            elif traffic_light_most_common == 'green':
                traffic_light_condition = False
            
            if traffic_light_condition == True :
                if stopline_center > 420:
                    adjust_flag = True
                    stop_process()
            else:
                if current_state == 'stop' :
                    ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99') #恢复直行 
                    current_state = 'straight'
                    adjust_flag = False
            
            if uturn_center > 400: #如果uturn 距离足够近
                uturn_process()
            
            if not adjust_flag: 
                if yellowsoildline_center < 190:
                    adjust_left_process()
                if yellowsoildline_center > 210:
                    adjust_right_process()
                    
        LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt

def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))
    detector()  # 调用之前定义的函数


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
    