import pdb
import numpy as np
import os
import os.path as osp
import json
import cv2
import colorsys
import random
from collections import namedtuple


BBox = namedtuple('BBox', ['cate', 'cate_index', 
                            'tracking_id', 'tracking_age', 
                            'uv_coord', 'pos_filter', 'obstacle_width', 'obstacle_length'])

def gendir(path):
    if not osp.exists(path):
        os.makedirs(path)
    return path

def get_json_info(json_path):
    jfile = open(json_path, 'r')
    try:
        jinfo = json.load(jfile)
    except:
        print(json_path)
        return None
    else:
        jfile.close()
        return jinfo

def gen_color_list():
    hsv_tuples = [(1.0 * x / 22, 1., 1.) for x in range(22)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0]*255), int(x[1]*255), int(x[2]*255)), colors))
    random.seed(0)
    random.shuffle(colors)
    random.seed(None)
    return colors

def index2cate(idx):
    assert(idx >= 0)
    namedict = {
        0: 'car', 1: 'truck', 2: 'bus', 
        3: 'pedestrian', 4: 'bicycle', 
        5: 'motorcycle', 6: 'tricycle', 
        7: 'misc'
    }
    return namedict[idx]

##### parse image and json infomation functions ####
def img_preprocess(ori_img, desay_crop_top=0, desay_crop_bottom=0, nm_crop_top=200, nm_crop_bottom=120):
    raw_width = 1920
    raw_height = 1280
    dst_width = 512
    dst_height = 256

    desay_img = ori_img[desay_crop_top:(raw_height-desay_crop_bottom), :]
    nm_img = desay_img[nm_crop_top:(raw_height-desay_crop_bottom-nm_crop_bottom), :]
    dst_img = cv2.resize(nm_img, (dst_width, dst_height))
    return dst_img

def parse_track_info(obj):
    uvbbox = obj['uv_bbox2d']
    topx = uvbbox['obstacle_bbox.x']
    topy = uvbbox['obstacle_bbox.y']
    w = uvbbox['obstacle_bbox.width']
    h = uvbbox['obstacle_bbox.height']
    uvcoord = [topx, topy, w, h]
    obj_id = obj['obstacle_id']
    obj_age = obj['obstacle_age']
    cate_index = obj['obstacle_type']
    cate_name = index2cate(cate_index-1)
    pos_filter = obj['position']
    pos_x = pos_filter['obstacle_pos_x_filter']
    pos_y = pos_filter['obstacle_pos_y_filter']
    position = [pos_x, pos_y]
    obstacle_width = obj['obstacle_width']
    obstacle_length = obj['obstacle_length']
    bbox_info = BBox(cate=cate_name, cate_index=cate_index, 
                        tracking_id=obj_id, tracking_age=obj_age, 
                        uv_coord=uvcoord, pos_filter=position, 
                        obstacle_width=obstacle_width, obstacle_length=obstacle_length)
    return bbox_info

def process_image_info(imgdir, jsondir, filename, crop_config):
    imgpath = osp.join(imgdir, filename)
    jsonname = filename.replace('.png', '.json')
    jsonpath = osp.join(jsondir, jsonname)

    desay_corp_top, desay_crop_bottom, nm_crop_top, nm_crop_bottom = crop_config

    ori_img = cv2.imread(imgpath)
    img = img_preprocess(ori_img, desay_corp_top, desay_crop_bottom, nm_crop_top, nm_crop_bottom)
    jsoninfo = get_json_info(jsonpath)
    bbox_infos = []
    try:
        tracks = jsoninfo['tracks']
        if isinstance(tracks, list):
            for track in tracks:
                bbox_info = parse_track_info(track)
                bbox_infos.append(bbox_info)
    except:
        print("null")
    return img, bbox_infos

def process_bev_info(jsondir, filename):
    jsonname = filename.replace('.png', '.json')
    jsonpath = osp.join(jsondir, jsonname)
    jsoninfo = get_json_info(jsonpath)
    bbox_infos = []
    try:
        tracks = jsoninfo['tracks']
        if isinstance(tracks, list):
            for track in tracks:
                bbox_info = parse_track_info(track)
                bbox_infos.append(bbox_info)
    except:
        print("null")
    return bbox_infos
##### parse image and json infomation functions ####

##### draw infomation functions ####
def draw_all_bbox_per_img(img, bbox_infos, camera_position, frame_id, show_id=False):
    color_list = gen_color_list()
    h, w, _ = img.shape
    font = cv2.FONT_HERSHEY_SIMPLEX
    c_size = cv2.getTextSize(camera_position, 0, 0.5, 1)[0]
    cv2.putText(img, camera_position, (0, c_size[1]), font, 0.5, (0, 0, 229), 1)
    if show_id:
        frame_msg = 'frame %d' % frame_id
        f_size_w, f_size_h = cv2.getTextSize(frame_msg, 0, 0.5, 1)[0]
        cv2.putText(img, frame_msg, (w-f_size_w, f_size_h), font, 0.5, (0, 0, 229), 1)

    for bbox_info in bbox_infos:
        coord = bbox_info.uv_coord
        left_top = (int(coord[0]), int(coord[1]))
        left_bottom = (int(coord[0]), int(coord[1]+coord[3]))
        right_bottom = (int(coord[0]+coord[2]), int(coord[1]+coord[3]))
        category = bbox_info.cate
        cate_index = bbox_info.cate_index
        tracking_id = bbox_info.tracking_id
        tracking_age = bbox_info.tracking_age
        bbox_msg = '%d%s |a %d' % (tracking_id, category, tracking_age)

        bbox_color = color_list[cate_index]
        bbox_thick = 1
        fontScale = 0.4
        cv2.rectangle(img, left_top, right_bottom, bbox_color, bbox_thick)
        cv2.putText(img, bbox_msg, (left_bottom[0],left_bottom[1]-3), font, fontScale, bbox_color, thickness=bbox_thick, lineType=cv2.LINE_AA)

    return img

def draw_one_track_bbox(img, bbox_infos, camera_position, frame_id, dst_tracking_id=[1], show_id=False):
    color_list = gen_color_list()
    h, w, _ = img.shape
    font = cv2.FONT_HERSHEY_SIMPLEX
    c_size = cv2.getTextSize(camera_position, 0, 0.5, 1)[0]
    cv2.putText(img, camera_position, (0, c_size[1]), font, 0.5, (0, 0, 229), 1)
    if show_id:
        frame_msg = 'frame %d' % frame_id
        f_size_w, f_size_h = cv2.getTextSize(frame_msg, 0, 0.5, 1)[0]
        cv2.putText(img, frame_msg, (w-f_size_w, f_size_h), font, 0.5, (0, 0, 229), 1)

    for bbox_info in bbox_infos:
        tracking_id = bbox_info.tracking_id
        if tracking_id not in dst_tracking_id:
            continue
        coord = bbox_info.uv_coord
        left_top = (int(coord[0]), int(coord[1]))
        left_bottom = (int(coord[0]), int(coord[1]+coord[3]))
        right_bottom = (int(coord[0]+coord[2]), int(coord[1]+coord[3]))
        category = bbox_info.cate
        cate_index = bbox_info.cate_index
        tracking_age = bbox_info.tracking_age
        bbox_msg = '%d%s |a %d' % (tracking_id, category, tracking_age)

        bbox_color = color_list[cate_index]
        bbox_thick = 1
        fontScale = 0.4
        cv2.rectangle(img, left_top, right_bottom, bbox_color, bbox_thick)
        cv2.putText(img, bbox_msg, (left_bottom[0],left_bottom[1]-3), font, fontScale, bbox_color, thickness=bbox_thick, lineType=cv2.LINE_AA)

    return img

def draw_all_bev(bbox_infos, bev_range_config=(60, 40), ego_car_size=(20, 40), bev_car_size=(20, 50), scale=0.1):
    color_list = gen_color_list()
    font = cv2.FONT_HERSHEY_SIMPLEX
    bev_img_h = int(bev_range_config[0] / scale)
    bev_img_w = int(bev_range_config[1] / scale)
    bev_img = np.zeros((bev_img_h, bev_img_w, 3), np.uint8)
    bev_img.fill(50)
    cv2.line(bev_img, (bev_img_w-1, 0), (bev_img_w-1, bev_img_h-1), (255,255,255), 2)
    
    pix_lane_width = 3.5 / scale
    for i in range(3):
        cv2.line(bev_img, (int(bev_img_w/2-pix_lane_width*0.5*(2*i+1)), 0),
                        (int(bev_img_w/2-pix_lane_width*0.5*(2*i+1)), int(bev_img_h-1)),
                        (30+int(i*10), 150, 80), 2)
        cv2.line(bev_img, (int(bev_img_w/2+pix_lane_width*0.5*(2*i+1)), 0),
                        (int(bev_img_w/2+pix_lane_width*0.5*(2*i+1)), int(bev_img_h-1)),
                        (30+int(i*10), 150, 80), 2)
    ego_car_size = ego_car_size #pixel w, pixel h, 20,40
    eog_img = cv2.resize(cv2.imread('./material/ego_car.jpeg'), ego_car_size)
    bev_img[int(bev_img_h/2):int(bev_img_h/2+ego_car_size[1]), int(bev_img_w/2-ego_car_size[0]/2):int(bev_img_w/2+ego_car_size[0]/2)] = eog_img
    for bbox_info in bbox_infos:
        position = bbox_info.pos_filter
        bev_box_w, bev_box_l = bev_car_size # 20, 50, flexiable
        bev_cent_u = (-position[1]+bev_range_config[1]/2) / 0.1
        bev_cent_v = (bev_range_config[0]/2 - position[0]) / 0.1
        cate_index = bbox_info.cate_index
        tracking_id = bbox_info.tracking_id
        bbox_msg = '%d| %.2f, %.2f' % (tracking_id, position[1], position[0]) #dy, dx
        bbox_thick = 1
        fontScale = 0.4
        if cate_index > 3: # not car, =model_cate_num+1
            color = (0, 255, 255)
            radiu = 5
        else:
            color = color_list[cate_index]
            radiu = 10
        if (0<bev_cent_u<bev_img_w and 0<bev_cent_v<bev_img_h):
            cv2.circle(bev_img, (int(bev_cent_u), int(bev_cent_v)), radiu, color, -1)
            if bev_cent_u > bev_img_w/2:
                bev_show_x = int(bev_cent_u-bev_box_w/2)
            else:
                bev_show_x = int(bev_cent_u+bev_box_w -20)
            if bev_cent_v > bev_img_h/2:
                bev_show_y = int(bev_cent_v-bev_box_l/2 +10)
            else:
                bev_show_y = int(bev_cent_v+bev_box_l/2)
            cv2.putText(bev_img, bbox_msg, (bev_show_x, bev_show_y), font, fontScale, (255,255,255), thickness=bbox_thick, lineType=cv2.LINE_AA)
    bev_img = cv2.resize(bev_img, (341, 512)) # (400,600) -> (xxx, 512)
    return bev_img

def draw_one_track_bev(bbox_infos, dst_tracking_id=[1], bev_range_config=(60, 40), ego_car_size=(20, 40), bev_car_size=(20, 50), scale=0.1):
    color_list = gen_color_list()
    font = cv2.FONT_HERSHEY_SIMPLEX
    bev_img_h = int(bev_range_config[0] / scale)
    bev_img_w = int(bev_range_config[1] / scale)
    bev_img = np.zeros((bev_img_h, bev_img_w, 3), np.uint8)
    bev_img.fill(50)
    cv2.line(bev_img, (bev_img_w-1, 0), (bev_img_w-1, bev_img_h-1), (255,255,255), 2)
    
    pix_lane_width = 3.5 / scale
    for i in range(3):
        cv2.line(bev_img, (int(bev_img_w/2-pix_lane_width*0.5*(2*i+1)), 0),
                        (int(bev_img_w/2-pix_lane_width*0.5*(2*i+1)), int(bev_img_h-1)),
                        (30+int(i*10), 150, 80), 2)
        cv2.line(bev_img, (int(bev_img_w/2+pix_lane_width*0.5*(2*i+1)), 0),
                        (int(bev_img_w/2+pix_lane_width*0.5*(2*i+1)), int(bev_img_h-1)),
                        (30+int(i*10), 150, 80), 2)
    ego_car_size = ego_car_size #pixel w, pixel h, 20,40
    eog_img = cv2.resize(cv2.imread('./material/ego_car.jpeg'), ego_car_size)
    bev_img[int(bev_img_h/2):int(bev_img_h/2+ego_car_size[1]), int(bev_img_w/2-ego_car_size[0]/2):int(bev_img_w/2+ego_car_size[0]/2)] = eog_img
    for bbox_info in bbox_infos:
        tracking_id = bbox_info.tracking_id
        if tracking_id not in dst_tracking_id:
            continue
        position = bbox_info.pos_filter
        bev_box_w, bev_box_l = bev_car_size # 20, 50, flexiable
        bev_cent_u = (-position[1]+bev_range_config[1]/2) / 0.1
        bev_cent_v = (bev_range_config[0]/2 - position[0]) / 0.1
        cate_index = bbox_info.cate_index
        bbox_msg = '%d| %.2f, %.2f' % (tracking_id, position[1], position[0]) #dy, dx
        bbox_thick = 1
        fontScale = 0.4
        if cate_index > 3:
            color = (0, 255, 255)
            radiu = 5
        else:
            color = color_list[cate_index]
            radiu = 10
        if (0<bev_cent_u<bev_img_w and 0<bev_cent_v<bev_img_h):
            cv2.circle(bev_img, (int(bev_cent_u), int(bev_cent_v)), radiu, color, -1)
            if bev_cent_u > bev_img_w/2:
                bev_show_x = int(bev_cent_u-bev_box_w/2)
            else:
                bev_show_x = int(bev_cent_u+bev_box_w -20)
            if bev_cent_v > bev_img_h/2:
                bev_show_y = int(bev_cent_v-bev_box_l/2 +10)
            else:
                bev_show_y = int(bev_cent_v+bev_box_l/2)
            cv2.putText(bev_img, bbox_msg, (bev_show_x, bev_show_y), font, fontScale, (255,255,255), thickness=bbox_thick, lineType=cv2.LINE_AA)
    bev_img = cv2.resize(bev_img, (341, 512)) # (400,600) -> (xxx, 512)
    return bev_img

##### draw infomation functions ####