import cv2
from tqdm import tqdm
from util.operation import YOLO
import time

model = YOLO(onnx_path=r'../common/dect_robot.onnx')
# 框类别文字
bbox_labelstr = {
    'font_size': 1,         # 字体大小
    'font_thickness': 1,   # 字体粗细
    'offset_x': 0,          # X 方向，文字偏移距离，向右为正
    'offset_y': -80,        # Y 方向，文字偏移距离，向下为正
}


def process_frame(img_bgr):
    """
    输入摄像头画面 bgr-array，输出图像 bgr-array
    """

    t0 = time.perf_counter()
    det = model.decect(img_bgr)
    t1 = time.perf_counter()
    # {'bbox': [57, 390, 207, 882], 'classes': 'person', 'conf':0.88}
    for i in range(len(det)):
        bbox_xyxy = det[i]['bbox']
        img_bgr = cv2.rectangle(img_bgr, (bbox_xyxy[0], bbox_xyxy[1]), (bbox_xyxy[2], bbox_xyxy[3]),
                                (0, 0, 255),
                                3)
        # 写框类别文字：图片，文字字符串，文字左上角坐标，字体，字体大小，颜色，字体粗细
        img_bgr = cv2.putText(img_bgr, det[i]["classes"], (bbox_xyxy[0]+bbox_labelstr['offset_x'], bbox_xyxy[1]+bbox_labelstr['offset_y']), cv2.FONT_HERSHEY_SIMPLEX, bbox_labelstr['font_size'], (0, 0, 255), bbox_labelstr['font_thickness'])
        # 写框置信度
        img_bgr = cv2.putText(img_bgr, '{:.2f}'.format(det[i]["conf"]), (bbox_xyxy[0]-bbox_labelstr['offset_x'], bbox_xyxy[1]-bbox_labelstr['offset_y']), cv2.FONT_HERSHEY_SIMPLEX, bbox_labelstr['font_size'], (0, 0, 255), bbox_labelstr['font_thickness'])
        # 打印速度
        fps = 1.0 / (t1 - t0)
        img_bgr = cv2.putText(img_bgr, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);
    return img_bgr


def generate_video(input_path='videos/robot.mp4'):
    filehead = input_path.split('/')[-1].split('.')[0]
    output_path = "out-" + filehead + '.mp4'

    print('视频开始处理', input_path)

    # 获取视频总帧数
    cap = cv2.VideoCapture(input_path)
    frame_count = 0
    while cap.isOpened():
        success, frame = cap.read()
        frame_count += 1
        if not success:
            break
    cap.release()
    print('视频总帧数为', frame_count)

    # cv2.namedWindow('Crack Detection and Measurement Video Processing')
    cap = cv2.VideoCapture(input_path)
    frame_size = (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = cap.get(cv2.CAP_PROP_FPS)

    out = cv2.VideoWriter(output_path, fourcc, fps, (int(frame_size[0]), int(frame_size[1])))

    # 进度条绑定视频总帧数
    with tqdm(total=frame_count - 1) as pbar:
        try:
            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    break
                try:
                    frame = process_frame(frame)
                except Exception as error:
                    print('报错！', error)
                    pass

                if success:
                    # cv2.imshow('Video Processing', frame)
                    out.write(frame)
                    # 进度条更新一帧
                    pbar.update(1)
        except:
            print('中途中断')
            pass
    cv2.destroyAllWindows()
    out.release()
    cap.release()
    print('视频已保存', output_path)


if __name__ == "__main__":
    generate_video(input_path="../videos/45.mp4")