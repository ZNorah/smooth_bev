from copy import deepcopy
import xxlimited
from replay_utils import *

### update view trackers ###
def is_box_show(bbox_info, bev_range_config=(60, 40), scale=0.1):
    show_flag = True
    position = bbox_info.pos_filter
    bev_cent_u = (-position[1]+bev_range_config[1]/2) / scale
    bev_cent_v = (bev_range_config[0]/2 - position[0]) / scale
    if (0<bev_cent_u<bev_range_config[1]/scale and 0<bev_cent_v<bev_range_config[0]/scale):
        show_flag = True
    else:
        show_flag = False
    ahead_flag = is_ahead(bbox_info)
    if ahead_flag:
        show_flag = False
    return show_flag

def is_ahead(bbox_info):
    ahead_flag = False
    ahead_range = 2.5#2.0
    position = bbox_info.pos_filter
    dis_h, dis_w = position #[dx, dy]
    if dis_h > 0: #ahead
        if abs(dis_w) < ahead_range / 2:
            ahead_flag = True
            print('%d Too ahead' % bbox_info.tracking_id)
    return ahead_flag

def update_trackers(trackers:dict, fusion_bbox_infos, bev_range_config=(60, 40), scale=0.1, max_length_=8, vanish_length_=8):
    first_appear_ids = []
    for bbox_info in fusion_bbox_infos:
        bbox_show_flag = is_box_show(bbox_info, bev_range_config, scale) # show in bev range
        tracking_id = bbox_info.tracking_id
        if tracking_id not in trackers: # id first time appearance
            # TODO: 是否是远处车辆id跳变或者大车id跳变导致的？是否需要修复id进行显示否则会突然消失？还是将id跳变的tracker继承？
            # now:新出现的id如果跟之前的id iou scale太大，可以进行匹配和发出
            # init tracker info
            first_appear_ids.append(tracking_id)
            track_info = {"info": bbox_info, "focus_time":1,
                          "missing_time":0, "match_flag":True,
                          "fill_flag":False, "smooth_cate": -1,
                          "link_flag":-22}
            if not bbox_show_flag:
                track_info['focus_time'] = 0
                track_info['match_flag'] = False
            trackers[tracking_id] = [track_info]
        elif tracking_id in trackers: #detected in cur frame or ever detected in 8 frames
            if len(trackers[tracking_id]) < 3: # 0,1|  2 frames cache
                first_appear_ids.append(tracking_id)
            track_info = deepcopy(trackers[tracking_id][-1])
            bbox_info_new = BBox(cate=bbox_info.cate, cate_index=bbox_info.cate_index,
                                        tracking_id=bbox_info.tracking_id, tracking_age=track_info['info'].tracking_age+1,
                                        uv_coord=bbox_info.uv_coord, pos_filter=bbox_info.pos_filter,
                                        obstacle_width=bbox_info.obstacle_width, obstacle_length=bbox_info.obstacle_length,
                                        alpha=bbox_info.alpha, heading_yaw=bbox_info.heading_yaw)
            track_info["info"] = bbox_info_new
            track_info['fill_flag'] = False
            track_info['smooth_cate'] = -1
            track_info['link_flag'] = trackers[tracking_id][-1]['link_flag'] #onece linked, always log it?
            if bbox_show_flag:
                track_info["focus_time"] += 1
                track_info["match_flag"] = True
            else:
                track_info["focus_time"] = 0
                track_info['match_flag'] = False # detected but not show
            trackers[tracking_id].append(track_info)
            if len(trackers[tracking_id]) > max_length_:
                del trackers[tracking_id][0]
    for tracking_id, track_info in trackers.items():
        if not track_info[-1]['match_flag']: #not det or det but not show in this frame, if not det, age not +1
            trackers[tracking_id][-1]['missing_time'] += 1
            trackers[tracking_id][-1]['focus_time'] = 0
        else:
            trackers[tracking_id][-1]['match_flag'] = False
            trackers[tracking_id][-1]['missing_time'] = 0

    for tid, tinfos in trackers.items():
        if tinfos[-1]['focus_time'] < 2 and tid not in first_appear_ids: #0, 1, after miss, 2 frames cache
            first_appear_ids.append(tid)
    #first appear check
    for first_id in first_appear_ids:
        first_tracker = trackers[first_id][-1]
        if first_tracker['focus_time'] == 0: #cur show
            continue
        first_cate = first_tracker['info'].cate_index
        if first_cate > 3: # only for car
            continue
        for tid, tracker_info in trackers.items(): # first id pair with id in trackers
            # if first_id == 61 and tid == 33:
            #     pdb.set_trace()
            if tid == first_id:
                continue
            if len(tracker_info) < 3: #old detected id
                continue
            cur_lx, cur_ly, cur_width, cur_height = track_info[-1]['info'].uv_coord
            if (abs(cur_lx)<10 or abs(cur_lx+cur_width-512)<10) or (cur_height/cur_width)>1.5 or (cur_height<256/5 or cur_width<512/5):
                continue
            cur_cate = tracker_info[-1]['info'].cate_index
            cur_focus_time = max(tracker_info[-2]['focus_time'], tracker_info[-3]['focus_time']) #cur miss, 才有可能被first id替换
            inter_scale_thre = 0.22
            diff_w_scale_thre = 0.4
            if cur_focus_time >= max_length_/2:
                inter_scale, diff_w_scale = bev_view_overlap(first_tracker['info'], tracker_info[-1]['info'])
                if inter_scale > 0:
                    print('fisrt id: %d , overlap id: %d, overlap_scale: %f, diff_scale: %f' % (first_id, tid, inter_scale, diff_w_scale))
                    # if first_id==59 and tid==33:
                    #     pdb.set_trace()
                # if inter_scale > inter_scale_thre and diff_w_scale < diff_w_scale_thre:
                if diff_w_scale < diff_w_scale_thre/2:
                    inter_scale_thre /= 2
                replace_flag = False
                if inter_scale > inter_scale_thre and diff_w_scale < diff_w_scale_thre and (trackers[tid][-1]['link_flag']<0 or trackers[tid][-1]['link_flag']==first_id):
                    replace_flag = True
                
                if replace_flag:
                    # if first_id == 33 and tid == 61:
                    #     for item in trackers[33]:
                    #         print(item)
                    #     for item in trackers[61]:
                    #         print(item)
                    if tracker_info[-1]['missing_time'] == 0:
                        trackers[first_id][-1]["missing_time"] += 1 #前面的没有漏检，初次检测到的重复框记miss
                    elif tracker_info[-1]['missing_time'] < 4:
                        print('%d maybe redetected in %d' % (first_id, tid))
                        print('%d replace %d' % (first_id, tid)) #前面的漏检了，当前检测到的继承它的focus信息，并且将之前的从tracker中删除
                        trackers[first_id][-1]["focus_time"] = max(cur_focus_time, trackers[first_id][-1]["focus_time"]) + 1
                        trackers[first_id][-1]["missing_time"] = 0

                        last_bbox_info = trackers[tid][-1]['info']
                        last_alpha = last_bbox_info.alpha
                        last_yaw = last_bbox_info.heading_yaw
                        for iii, item in enumerate(trackers[first_id]):
                            last_first_info = item['info']
                            new_bbox = BBox(cate=last_first_info.cate, cate_index=last_first_info.cate_index,
                                            tracking_id=last_first_info.tracking_id, tracking_age=last_first_info.tracking_age,
                                            uv_coord=last_first_info.uv_coord, pos_filter=last_first_info.pos_filter,
                                            obstacle_width=last_first_info.obstacle_width, obstacle_length=last_first_info.obstacle_length, 
                                            alpha=last_alpha, heading_yaw=last_yaw)
                            trackers[first_id][iii]['info'] = new_bbox

                        trackers[tid][-1]['missing_time'] += 1
                        trackers[tid][-1]['link_flag'] = first_id #log link car

                        trackers[first_id][-1]['link_flag'] = tid

    del_list = [] #delete obstacle missing too much times
    for tracking_id, track_info in trackers.items():
        if trackers[tracking_id][-1]['missing_time'] > vanish_length_:
            del_list.append(tracking_id)
    for tracking_id in del_list:
        if tracking_id in trackers:
            del trackers[tracking_id]
    return trackers

### update view trackers ###

### filter bev car ###
def if_publish(last_tracker_info:dict, delay_n:int):
    publish_flag = False
    if last_tracker_info['focus_time'] < (1+delay_n):
        publish_flag = False
    elif last_tracker_info['missing_time'] > 0:
        publish_flag = False
    else:
        publish_flag = True
    return publish_flag

def delay_n_filter(tracker_infos:list):
    """
    filter main function, add mothed in this function
    """
    delay_n = 1
    # out of w_range odd, delay more
    if abs(tracker_infos[-1]['info'].pos_filter[1]) > 12: # 
        delay_n += 1
    
    publish_flag = False
    last_tracker_info = tracker_infos[-1]
    tracking_id = tracker_infos[-1]['info'].tracking_id

    #TODO: more condition

    # ped，not car, delay more， delay_n +=1
    if last_tracker_info['info'].cate_index > 3:
        delay_n += 1

    delay_n = is_ahead_in_middle(tracker_infos, delay_n)

    # 1. simple focus and missing times
    middle_range = (18, 5) # h:18m, w:5m
    publish_flag = if_publish(last_tracker_info, delay_n)

    small_delay_n = is_not_middle_and_small(tracker_infos, delay_n, middle_range)
    # if publish_flag:
    if small_delay_n > delay_n:
        # print('tid:', tracking_id, 'far and bbox not big!')
        publish_flag = if_publish(last_tracker_info, small_delay_n)

    # 2. burst in the middle view
    burst_delay_n = is_burst_in_middle_view(tracker_infos, small_delay_n, middle_range)
    if publish_flag:
        # if burst_delay_n > delay_n:
        if burst_delay_n > small_delay_n:
            print('tid: ', tracking_id, 'burst in!')
            publish_flag = if_publish(last_tracker_info, burst_delay_n)

    flicker_delay_n = is_flicker(tracker_infos, burst_delay_n, middle_range)
    if publish_flag:
        if flicker_delay_n > burst_delay_n:
            print('tid: ', tracking_id, 'flicker delay more: ', flicker_delay_n)
            publish_flag = if_publish(last_tracker_info, flicker_delay_n)
    # if tracking_id == 61:
    #     print('delay_n', delay_n)
    #     print('small delay', small_delay_n)
    #     print('burst delay', burst_delay_n)
    #     print('flick delay', flicker_delay_n)
    #     print('focus', tracker_infos[-1]['focus_time'])
    #     print(publish_flag)

    return publish_flag

def is_flicker(tracker_infos:list, delay_n=1, middle_range=(15, 5)):
    new_delay_n = delay_n
    arround_h, arround_w = middle_range
    pos_h, pos_w = tracker_infos[-1]['info'].pos_filter
    if abs(pos_h) < arround_h and abs(pos_w) < arround_w:
        return new_delay_n # too close
    else:
        missing_list = []
        for track in tracker_infos:
            missing_time = track['missing_time']
            missing_list.append(missing_time)
        non_zero_num = 0
        for ii in range(len(missing_list)):
            miss_value = missing_list[ii]
            if ii == 0:
                if miss_value == 0:
                    start_zero_flag = True
                else:
                    start_zero_flag = False
            if miss_value ==1:
                non_zero_num += 1
        if start_zero_flag:
            new_delay_n += non_zero_num #[0010010]->2   /[001234500]
        else:
            new_delay_n += (non_zero_num + 1) #[100100]->3
        return new_delay_n

def is_burst_in_middle_view(tracker_infos:list, delay_n=1, middle_range=(18, 5)):
    burst_delay_n = delay_n
    pos_h, pos_w = tracker_infos[-1]['info'].pos_filter #pos_x ->zongxinag  pos_y->hengxiang
    coord = tracker_infos[-1]['info'].uv_coord
    left_x = coord[0]
    right_x = left_x + coord[2]
    if abs(pos_h) < middle_range[0]: #h inside 18m
        if tracker_infos[-1]['focus_time'] > (delay_n+2) or abs(pos_w) < middle_range[1]:
            return burst_delay_n #apperence more than delay_n, or too close to left/right side
        elif abs(right_x - 512) < 10 or abs(left_x) < 10:
            return burst_delay_n
        else:
            burst_delay_n += 1
            return burst_delay_n #else is suddenly appearence
    else:
        return burst_delay_n

def is_not_middle_and_small(tracker_infos:list, delay_n=1, middle_range=(18, 5)):
    #only for vehical
    tracking_id = tracker_infos[-1]['info'].tracking_id
    small_delay_n = delay_n
    pos_h, pos_w = tracker_infos[-1]['info'].pos_filter #pos_x ->zongxiang  pos_y->hengxiang
    coord = tracker_infos[-1]['info'].uv_coord
    width, height = coord[2:]
    left_x = coord[0]
    right_x = left_x + width
    
    if len(tracker_infos) < 6:
        if abs(pos_h) > middle_range[0] and width*height < 50*50:
            if height/width > 2.5 and (abs(left_x) < 10 or abs(right_x - 512) < 10): #边缘截断，形状不正常的car
                small_delay_n += 3
            elif height < 256/6 or height/width>2.5:
                small_delay_n += 3
            else:
                small_delay_n += 1
        else:
            if (height < 256/4) and (width/height > 3):
                small_delay_n += 2
            elif abs(left_x) < 10 or abs(right_x-512)<10:
                small_delay_n += 1
    else:
        if (height < 256/20 or width < 512/20):
            small_delay_n += 3
        elif (height < 256/6) or (height/width > 2.5):
            small_delay_n += 3
        elif abs(left_x) < 10 or abs(right_x-512)<10:
            small_delay_n += 1
    if tracker_infos[-1]['info'].cate_index > 3:
        small_delay_n = delay_n
    return small_delay_n

def is_ahead_in_middle(tracker_infos:list, delay_n=1):
    ahead_delay_n = delay_n
    pos_h, pos_w = tracker_infos[-1]['info'].pos_filter
    ahead_range = 1.5
    if pos_h > 0 and abs(pos_w) < ahead_range:
        ahead_delay_n += 1
    return ahead_delay_n

### filter bev car ###

### smooth tracker ###

def bev_view_car_iou(bev1_info, bev2_info, bev_range_config=(60, 40), scale=0.1):
    position1 = bev1_info.pos_filter
    position2 = bev2_info.pos_filter
    cate1 = bev1_info.cate_index
    cate2 = bev2_info.cate_index

    if cate1 > 3:
        bev1_box_w = 10; bev1_box_l = 10 # draw size
    else:
        bev1_box_w = bev1_info.obstacle_width/scale; bev1_box_l = bev1_info.obstacle_length/scale
    if cate2 > 3:
        bev2_box_w = 10; bev2_box_l = 10
    else:
        bev2_box_w = bev2_info.obstacle_width/scale; bev2_box_l = bev2_info.obstacle_length/scale
    bev1_box_area = bev1_box_w * bev1_box_l
    bev2_box_area = bev2_box_w * bev2_box_l

    bev_range_h, bev_range_w = bev_range_config
    bev1_cent_u = (-position1[1]+bev_range_w/2) / scale; bev1_cent_v = (bev_range_h/2 - position1[0]) / scale
    bev1_left_x = bev1_cent_u - bev1_box_w / 2; bev1_right_x = bev1_cent_u + bev1_box_w / 2
    bev1_top_y = bev1_cent_v - bev1_box_l / 2; bev1_bot_y = bev1_cent_v + bev1_box_l / 2
    bev2_cent_u = (-position2[1]+bev_range_w/2) / scale; bev2_cent_v = (bev_range_h/2 - position2[0]) / scale
    bev2_left_x = bev2_cent_u - bev2_box_w / 2; bev2_right_x = bev2_cent_u + bev2_box_w / 2
    bev2_top_y = bev2_cent_v - bev2_box_l / 2; bev2_bot_y = bev2_cent_v + bev2_box_l / 2

    #cal iou
    left_top_x = max(bev1_left_x, bev2_left_x)
    left_top_y = max(bev1_top_y, bev2_top_y)
    right_bot_x = min(bev1_right_x, bev2_right_x)
    right_bot_y = min(bev1_bot_y, bev2_bot_y)
    inter_w = max(right_bot_x - left_top_x, 0)
    inter_h = max(right_bot_y - left_top_y, 0)
    inter_area = inter_w * inter_h
    union_area = bev1_box_area + bev2_box_area - inter_area
    iou = inter_area / union_area

    return iou

def bbox_overlap(bev1_info, bev2_info):
    bbox1_left_x, bbox1_top_y, bbox1_box_w, bbox1_box_h = bev1_info.uv_coord
    bbox2_left_x, bbox2_top_y, bbox2_box_w, bbox2_box_h = bev2_info.uv_coord
    bbox1_right_x = bbox1_left_x + bbox1_box_w
    bbox1_bot_y = bbox1_top_y + bbox1_box_h
    bbox2_right_x = bbox2_left_x + bbox2_box_w
    bbox2_bot_y = bbox2_top_y + bbox2_box_h

    bbox1_area = bbox1_box_w * bbox1_box_h
    bbox2_area = bbox2_box_w * bbox2_box_h

    left_top_x = max(bbox1_left_x, bbox2_left_x)
    left_top_y = max(bbox1_top_y, bbox2_top_y)
    right_bot_x = min(bbox1_right_x, bbox2_right_x)
    right_bot_y = min(bbox1_bot_y, bbox2_bot_y)
    inter_w = max(right_bot_x - left_top_x, 0)
    inter_h = max(right_bot_y - left_top_y, 0)
    inter_area = inter_w * inter_h
    inter_scale1 = inter_area / bbox1_area
    inter_scale2 = inter_area / bbox2_area
    
    pos1 = bev1_info.pos_filter
    pos2 = bev2_info.pos_filter
    if (pos1[0]*pos2[0] > 0) and (pos1[1] * pos2[1] > 0):
        union_area = bbox1_area + bbox2_area - inter_area
        # max_inter_scale = inter_area / union_area
        # max_inter_scale = max(inter_scale1, inter_scale2)
        max_inter_scale = inter_scale1 #fillbox scale
    else:
        max_inter_scale = 0
    return max_inter_scale

def bev_view_overlap(bev1_info, bev2_info, bev_range_config=(60, 40), scale=0.1):
    position1 = bev1_info.pos_filter
    position2 = bev2_info.pos_filter
    cate1 = bev1_info.cate_index
    cate2 = bev2_info.cate_index

    if cate1 > 3:
        bev1_box_w = 10; bev1_box_l = 10 # draw size
    else:
        bev1_box_w = bev1_info.obstacle_width/scale; bev1_box_l = bev1_info.obstacle_length/scale # bev car size(flexible)
    if cate2 > 3:
        bev2_box_w = 10; bev2_box_l = 10
    else:
        bev2_box_w = bev2_info.obstacle_width/scale; bev2_box_l = bev2_info.obstacle_length/scale
    bev1_box_area = bev1_box_w * bev1_box_l
    bev2_box_area = bev2_box_w * bev2_box_l

    bev_range_h, bev_range_w = bev_range_config
    bev1_cent_u = (-position1[1]+bev_range_w/2) / scale; bev1_cent_v = (bev_range_h/2 - position1[0]) / scale
    bev1_left_x = bev1_cent_u - bev1_box_w / 2; bev1_right_x = bev1_cent_u + bev1_box_w / 2
    bev1_top_y = bev1_cent_v - bev1_box_l / 2; bev1_bot_y = bev1_cent_v + bev1_box_l / 2
    bev2_cent_u = (-position2[1]+bev_range_w/2) / scale; bev2_cent_v = (bev_range_h/2 - position2[0]) / scale
    bev2_left_x = bev2_cent_u - bev2_box_w / 2; bev2_right_x = bev2_cent_u + bev2_box_w / 2
    bev2_top_y = bev2_cent_v - bev2_box_l / 2; bev2_bot_y = bev2_cent_v + bev2_box_l / 2

    #cal iou
    left_top_x = max(bev1_left_x, bev2_left_x)
    left_top_y = max(bev1_top_y, bev2_top_y)
    right_bot_x = min(bev1_right_x, bev2_right_x)
    right_bot_y = min(bev1_bot_y, bev2_bot_y)
    inter_w = max(right_bot_x - left_top_x, 0)
    inter_h = max(right_bot_y - left_top_y, 0)
    inter_area = inter_w * inter_h
    inter_scale1 = inter_area / bev1_box_area
    inter_scale2 = inter_area / bev2_box_area
    max_inter_scale = max(inter_scale1, inter_scale2)

    diff_w = abs(bev1_cent_u - bev2_cent_u)
    diff_w_scale = min(diff_w/bev1_box_w, diff_w/bev2_box_w)

    return max_inter_scale, diff_w_scale

def fill_missing_car(tracker:list, publish_bbox_infos:list, trackers:dict):
    max_length_ = 8
    # fill_range=(28, 18)
    fill_range=(40, 18)
    max_missing_num = int(max_length_ / 2)
    if abs(tracker[-1]['info'].pos_filter[1]) > fill_range[1]/2 and abs(tracker[-1]['info'].pos_filter[0]) > fill_range[0]/2:
        max_missing_num = int(max_missing_num / 2)
    fill_flag = False

    cate = tracker[-1]['info'].cate_index
    tracking_id = tracker[-1]['info'].tracking_id
    tracking_age = tracker[-1]['info'].tracking_age #when miss, return to zero
    if cate > 3: # only for vehicle
        return tracker, trackers
    focus_time_list = []
    for track in tracker:
        focus_time_list.append(track['focus_time'])
    max_focus_time = max(focus_time_list)
    # if tracker[-1]['info'].uv_coord[2] > 512/2: #w > 256
    #     max_length_ = 6

    last_bbox_info = tracker[-1]['info']
    last_coord = last_bbox_info.uv_coord
    left_x = last_coord[0]
    last_width, last_height = last_coord[2:]
    right_x = left_x + last_width
    edge_flag = False
    if (abs(right_x-512)<10 or abs(left_x)<10) and (last_height < 256/5 or last_height/last_width>1.5):
        edge_flag = True
    if edge_flag:
        max_missing_num = int(max_missing_num / 2)
        # if last_height 
    pids = []
    for pb in publish_bbox_infos:
        pids.append(pb.tracking_id)

    if len(tracker)>1 and tracker[-2]['missing_time'] > 0:
        max_missing_num = int(max_missing_num / 2)

    if (last_width < 512/20 or last_height < 256/20 or last_height/last_width>3) and max_missing_num > 0:
        max_missing_num = 0

    if len(tracker) >= 2 and 0<tracker[-1]['missing_time'] <=max_missing_num:
        position = last_bbox_info.pos_filter #pos_x zongxinag , pos_y hengxiang
        position_last = tracker[-2]['info'].pos_filter
        new_pos_x = position[0] + (position[0] - position_last[0]) / tracker[-1]['missing_time']
        new_pos_y = position[1] + (position[1] - position_last[1]) / tracker[-1]['missing_time']
        new_position = [new_pos_x, new_pos_y]
        new_last_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length, 
                                        alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
        # if tracking_id == 33:
        #     print(last_width, last_height, last_height/last_width)
        #     pdb.set_trace()

    # print(tracker[-1]['missing_time'], max_missing_num)
    if (tracking_age>=max_length_) and (max_focus_time >= max_length_) and \
        (0< tracker[-1]['missing_time'] <= max_missing_num) \
            and (tracker[-1]['link_flag'] not in pids):# and not edge_flag: #not been replace
        position = last_bbox_info.pos_filter #pos_x zongxinag , pos_y hengxiang
        position_last = tracker[-2]['info'].pos_filter
        new_pos_x = position[0] + (position[0] - position_last[0]) / tracker[-1]['missing_time']
        max_det_dx = 2 #(120km/h / 3.6 = 33m/s / 8 = 4m/s/frame)
        if abs(new_pos_x - position[0])/tracker[-1]['missing_time'] > max_det_dx:
            new_pos_x = position[0] + max_det_dx * abs((position[0] - position_last[0]) / tracker[-1]['missing_time'])/((position[0] - position_last[0]) / tracker[-1]['missing_time'])
        new_pos_y = position[1] + (position[1] - position_last[1]) / tracker[-1]['missing_time']
        new_position = [new_pos_x, new_pos_y]
        new_last_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                        alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
        if abs(position[0]) < fill_range[0] and abs(position[1]) < fill_range[1]: #fill range
            overlap_id = -22
            max_overlap = 0.
            dst_diff_w_scale = -1
            for pbbox_info in publish_bbox_infos:
                bev_overlap, diff_w_scale = bev_view_overlap(new_last_bbox_info, pbbox_info, (100, 40), 0.1)
                if bev_overlap > max_overlap:
                    max_overlap = bev_overlap
                    overlap_id = pbbox_info.tracking_id
                    dst_diff_w_scale = diff_w_scale
            if max_overlap > 0.2 and dst_diff_w_scale < 0.4:
                pass
            else:
                overlap_id = -22

            if (overlap_id == -22):
                max_overlap = 0.
                for pbbox_info in publish_bbox_infos:
                    bbox_iou = bbox_overlap(new_last_bbox_info, pbbox_info)
                    if bbox_iou > max_overlap:
                        max_overlap = bbox_iou
                        overlap_id = pbbox_info.tracking_id
                if max_overlap > 0.8:
                    print('------fill box overlap' ,overlap_id, tracking_id, max_overlap)
                    pass
                else:
                    overlap_id = -22

            if overlap_id > 0: #overlap / id switch or occlusion
                print('%d replace %d, stop filling' % (overlap_id, tracking_id))
                tracker[-1]['fill_flag'] = False
                new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age+1,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                        alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
                tracker[-1]['info'] = new_bbox_info
            else:
                fill_flag = True
                # if last_bbox_info.tracking_id == 37:
                #     print('fill 37', position[0], new_pos_x)
                new_bbox_track = {}
                new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age+1,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                        alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
                new_bbox_track['info'] = new_bbox_info
                new_bbox_track['focus_time'] = tracker[-2]['focus_time'] + 1
                new_bbox_track['missing_time'] = tracker[-1]['missing_time']
                new_bbox_track['match_flag'] = False
                new_bbox_track['smooth_cate'] = -1
                new_bbox_track['link_flag'] = -22
                tracker[-1] = new_bbox_track
                new_bbox_track['fill_flag'] = fill_flag

        else: # out of filling range
            new_bbox_track = {}
            new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                    tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                    uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                    obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                    alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
            new_bbox_track['info'] = new_bbox_info
            new_bbox_track['focus_time'] = tracker[-1]['focus_time']
            new_bbox_track['missing_time'] = tracker[-1]['missing_time']
            new_bbox_track['match_flag'] = False
            new_bbox_track['smooth_cate'] = -1
            if tracker[-1]['link_flag'] in pids:
                new_bbox_track['link_flag'] = tracker[-1]['link_flag']
            else:
                new_bbox_track['link_flag'] = -22
            tracker[-1] = new_bbox_track
            tracker[-1]['fill_flag'] = fill_flag #false

    else: # don't meet filling conditions
        tracker[-1]['fill_flag'] = fill_flag #false
        if len(tracker) >= 2 and 0<tracker[-1]['missing_time'] <= max_missing_num:
            new_bbox_track = {}
            new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                    tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                    uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                    obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                    alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
            new_bbox_track['info'] = new_bbox_info
            new_bbox_track['focus_time'] = tracker[-1]['focus_time']
            new_bbox_track['missing_time'] = tracker[-1]['missing_time']
            new_bbox_track['match_flag'] = False
            new_bbox_track['smooth_cate'] = -1
            if tracker[-1]['link_flag'] in pids:
                new_bbox_track['link_flag'] = tracker[-1]['link_flag']
            else:
                new_bbox_track['link_flag'] = -22
            new_bbox_track['fill_flag'] = fill_flag #false
            tracker[-1] = new_bbox_track
    return tracker, trackers

def link_missing_ped(ped_trackers:list, trackers:dict):
    disappear_ped_list = []
    suddenly_appear_ped_list = []
    link_to_publish_ped_infos = []
    for ped_tracker in ped_trackers:
        if ped_tracker[-1]['info'].tracking_age > 4 and ped_tracker[-1]['missing_time'] == 1: #suddenly disappear
            disappear_ped_list.append(ped_tracker)
        # elif (ped_tracker[-1]['info'].tracking_age < 3) and (ped_tracker[-1]['missing_time'] == 0):
        elif (len(ped_tracker) < 3) and (ped_tracker[-1]['missing_time'] == 0) and (ped_tracker[-1]['link_flag'] < 0):
            suddenly_appear_ped_list.append(ped_tracker)
    for dis_ped_tracker in disappear_ped_list:
        position_now_y = dis_ped_tracker[-1]['info'].pos_filter[1]
        dis_ped_height = dis_ped_tracker[-1]['info'].uv_coord[3]
        link_id = -1
        min_height_scale = 1000
        for suddenly_ped_tracker in suddenly_appear_ped_list:
            sudden_ped_height = suddenly_ped_tracker[-1]['info'].uv_coord[3]
            height_scale = min(abs(sudden_ped_height-dis_ped_height) / dis_ped_height, \
                abs(sudden_ped_height-dis_ped_height) / sudden_ped_height)
            if (height_scale < 0.4):
                if height_scale < min_height_scale:
                    min_height_scale = height_scale
                    link_id = suddenly_ped_tracker[-1]['info'].tracking_id

        if link_id != -1:
            for suddenly_ped_tracker in suddenly_appear_ped_list:
                if suddenly_ped_tracker[-1]['info'].tracking_id == link_id:
                    sudden_pos_y = suddenly_ped_tracker[-1]['info'].pos_filter[1]
                    if (suddenly_ped_tracker[-1]['link_flag'] < 0) and (sudden_pos_y*position_now_y)>0:
                        last_bbox_info = suddenly_ped_tracker[-1]['info']
                        print('------- link %d with %d ------' % (dis_ped_tracker[-1]['info'].tracking_id, link_id))
                        new_bbox_track = {}
                        new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                                tracking_id=last_bbox_info.tracking_id, tracking_age=dis_ped_tracker[-1]['info'].tracking_age+1,
                                                uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                                                obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                                                alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
                        new_bbox_track['info'] = new_bbox_info
                        if len(dis_ped_tracker) == 1: #id switch too fast?
                            new_bbox_track['focus_time'] = dis_ped_tracker[-1]['focus_time'] + 1
                        else:
                            new_bbox_track['focus_time'] = dis_ped_tracker[-2]['focus_time'] + 1
                        new_bbox_track['missing_time'] = 0
                        new_bbox_track['match_flag'] = False
                        new_bbox_track['fill_flag'] = dis_ped_tracker[-1]['fill_flag']
                        new_bbox_track['smooth_cate'] = -1
                        new_bbox_track['link_flag'] = dis_ped_tracker[-1]['info'].tracking_id
                        suddenly_ped_tracker[-1] = new_bbox_track
                        link_to_publish_ped_infos.append(suddenly_ped_tracker)
                        
                        trackers[dis_ped_tracker[-1]['info'].tracking_id][-1]['link_flag'] = -22

    return link_to_publish_ped_infos, trackers

def cate_in_same_main_cate(cate_list):
    flag = True
    cate0 = cate_list[0]
    if cate0 > 3:
        main_cate = 1
    else:
        main_cate = 0
    for item in cate_list[1:]:
        if item > 3:
            cur_cate = 1
        else:
            cur_cate = 0
        if cur_cate != main_cate:
            flag = False
    return flag

def vote_cate(tracker:list, vote_lenth=5):
    if len(tracker) < vote_lenth: # nead enough infomation
        return tracker
    if tracker[-1]['missing_time'] > 0:
        return tracker
    cate_list = []
    refine_cate_list = []
    for track in tracker:
        bbox_info = track['info']
        refine_cate_list.append(bbox_info.cate_index)
        if track['smooth_cate'] > -1:
            cate = track['smooth_cate'] # ori cate
        else:
            cate = bbox_info.cate_index
        cate_list.append(cate)
    rough_same_flag = cate_in_same_main_cate(cate_list)
    same_flag = cate_in_same_main_cate(refine_cate_list)
    cate_num = {}
    cate_time = {}
    for ii in range(len(cate_list)):
        cate = cate_list[ii]
        cate_weight = 1 + 0.5 * ii
        if cate in cate_num:
            cate_num[cate] += cate_weight
        else:
            cate_num[cate] = cate_weight

        if cate in cate_time:
            cate_time[cate] += 1
        else:
            cate_time[cate] = 1

    max_cate = -1
    max_cate_weight = -1
    for cate in cate_num:
        if cate_num[cate] > max_cate_weight:
            max_cate = cate
            max_cate_weight = cate_num[cate]

    main_cate = refine_cate_list[0]
    if (not same_flag or not rough_same_flag):
        if main_cate in cate_time:
            if max_cate != main_cate:
                if cate_time[main_cate] > 2:
                    max_cate = main_cate

    if max_cate != tracker[-1]['info'].cate_index:
        print('~~~~~~~~~~~~~')
        print('tid: ', tracker[-1]['info'].tracking_id, 'cate switch: ', tracker[-1]['info'].cate_index, max_cate)
        for c in cate_num:
            print(c, cate_num[c])
        new_cate = index2cate(max_cate-1)
        last_bbox_info = tracker[-1]['info']
        new_bbox_track = {}
        new_bbox_info = BBox(cate=new_cate, cate_index=max_cate,
                            tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                            uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                            obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                            alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
        new_bbox_track['info'] = new_bbox_info
        new_bbox_track['focus_time'] = tracker[-1]['focus_time']
        new_bbox_track['missing_time'] = tracker[-1]['missing_time']
        new_bbox_track['match_flag'] = tracker[-1]['match_flag']
        new_bbox_track['fill_flag'] = tracker[-1]['fill_flag']
        new_bbox_track['link_flag'] = tracker[-1]['link_flag']
        new_bbox_track['smooth_cate'] = last_bbox_info.cate_index
        tracker[-1] = new_bbox_track #replace with the newest infomation
    return tracker

def protect_ego_car(bbox_info, bev_range_config, trackers):
    pos_filter = bbox_info.pos_filter
    dw_pixel = 0; dh_pixel = 0; protect_pixel_w = 5; protect_pixel_h = 5
    scale = 0.1
    bev_range_h, bev_range_w =bev_range_config # h, w
    bev_h = bev_range_h / scale
    bev_w = bev_range_w / scale
    ego_w, ego_h =(20, 40) # w, h
    position = pos_filter #[dx, dy]
    if bbox_info.cate_index > 3:
        bev_box_w = 10; bev_box_l = 10
    else:
        bev_box_w = bbox_info.obstacle_width / scale; bev_box_l = bbox_info.obstacle_length / scale # flexible
    bev_cent_u = (-position[1]+bev_range_w/2) / scale; bev_cent_v = (bev_range_h/2 - position[0]) / scale

    ego_left_x = bev_w/2 - ego_w/2; ego_right_x = bev_w/2 + ego_w/2
    ego_top_y = bev_h/2; ego_bot_y = bev_h/2 + ego_h
    bev_left_x = bev_cent_u - bev_box_w / 2; bev_right_x = bev_cent_u + bev_box_w / 2
    bev_top_y = bev_cent_v - bev_box_l / 2; bev_bot_y = bev_cent_v + bev_box_l / 2

    if ego_left_x <= bev_left_x <= ego_right_x and ego_top_y <= bev_top_y <= ego_bot_y:
        dw_pixel = ego_right_x - bev_left_x + protect_pixel_w # move to right
        dh_pixel = ego_bot_y - bev_top_y + protect_pixel_h # move down
    elif ego_left_x <= bev_right_x <= ego_right_x and ego_top_y <= bev_top_y <= ego_bot_y:
        dw_pixel = - (bev_right_x - ego_right_x + protect_pixel_w) # move to left
        dh_pixel = ego_bot_y - bev_top_y + protect_pixel_h # move down
    elif ego_left_x <= bev_left_x <= ego_right_x and ego_top_y <= bev_bot_y <= ego_bot_y:
        dw_pixel = ego_right_x - bev_left_x + protect_pixel_w # move to right
        dh_pixel = - (bev_bot_y - ego_top_y + protect_pixel_h) # move up
    elif ego_left_x <= bev_right_x <= ego_right_x and ego_top_y <= bev_bot_y <= ego_bot_y:
        dw_pixel = - (bev_right_x - ego_right_x + protect_pixel_w) # move to left
        dh_pixel = - (bev_bot_y - ego_top_y + protect_pixel_h) # move up
    elif ego_left_x <= bev_left_x <= ego_right_x and ego_top_y <= bev_top_y and ego_bot_y >= bev_top_y:
        dw_pixel = ego_right_x - bev_left_x + protect_pixel_w # move to right
    elif ego_left_x <= bev_right_x <= ego_right_x and ego_top_y <= bev_top_y and ego_bot_y >= bev_top_y:
        dw_pixel = - (bev_right_x - ego_right_x + protect_pixel_w) # move to left
    new_bev_cent_u = bev_cent_u + dw_pixel
    new_bev_cent_v = bev_cent_v + dh_pixel
    new_dy = bev_range_w/2 - new_bev_cent_u * scale
    new_dx = bev_range_h/2 - new_bev_cent_v * scale
    new_pos_filter = [new_dx, new_dy]
    if dw_pixel != 0 or dh_pixel != 0:
        print('protect ego tid: ', bbox_info.tracking_id)

    new_bbox_info = BBox(cate=bbox_info.cate, cate_index=bbox_info.cate_index,
                        tracking_id=bbox_info.tracking_id, tracking_age=bbox_info.tracking_age,
                        uv_coord=bbox_info.uv_coord, pos_filter=new_pos_filter,
                        obstacle_width=bbox_info.obstacle_width, obstacle_length=bbox_info.obstacle_length,
                        alpha=bbox_info.alpha, heading_yaw=bbox_info.heading_yaw)
    trackers[bbox_info.tracking_id][-1]['info'] = new_bbox_info
    return new_bbox_info, trackers

def sort_by_radiu(bbox_infos):
    return 1/bbox_infos.obstacle_length*bbox_infos.pos_filter[0]*bbox_infos.pos_filter[0] +bbox_infos.pos_filter[1]*bbox_infos.pos_filter[1]

def finetune_two_bev(bbox1_info, bbox2_info, bev_range_config, trackers): # 'x' / 'y'
    #1. just move TODO: move strategy
    #2. TODO:many car/moto
    bev_range_h, bev_range_w =bev_range_config # h, w
    if bbox1_info.cate_index > 3:
        bev1_box_w = 10; bev1_box_l = 10
    else:
        bev1_box_w = bbox1_info.obstacle_width/0.1; bev1_box_l = bbox1_info.obstacle_length/0.1
    if bbox2_info.cate_index > 3:
        bev2_box_w = 10; bev2_box_l = 10
    else:
        bev2_box_w = bbox2_info.obstacle_width/0.1; bev2_box_l = bbox2_info.obstacle_length/0.1

    bev1_cent_u = (-bbox1_info.pos_filter[1]+bev_range_w/2) / 0.1; bev1_cent_v = (bev_range_h/2 - bbox1_info.pos_filter[0]) / 0.1
    bev1_left_x = bev1_cent_u - bev1_box_w / 2; bev1_right_x = bev1_cent_u + bev1_box_w / 2
    bev1_top_y = bev1_cent_v - bev1_box_l / 2; bev1_bot_y = bev1_cent_v + bev1_box_l / 2

    bev2_cent_u = (-bbox2_info.pos_filter[1]+bev_range_w/2) / 0.1; bev2_cent_v = (bev_range_h/2 - bbox2_info.pos_filter[0]) / 0.1
    bev2_left_x = bev2_cent_u - bev2_box_w / 2; bev2_right_x = bev2_cent_u + bev2_box_w / 2
    bev2_top_y = bev2_cent_v - bev2_box_l / 2; bev2_bot_y = bev2_cent_v + bev2_box_l / 2

    diff_u = min(bev1_right_x, bev2_right_x) - max(bev1_left_x, bev2_left_x)
    diff_v = min(bev1_bot_y, bev2_bot_y) - max(bev1_top_y, bev2_top_y)

    dw_pixel = 0; dh_pixel = 0; protect_pixel_w = 5; protect_pixel_h = 5
    print('diff_u: %f   diff_v: %f' % (diff_u, diff_v)) #修改后不如修改前
    if (bev1_left_x <= bev2_right_x <= bev1_right_x) and (bev1_left_x <= bev2_left_x <= bev1_right_x):
        # up or down |1 |2 |2 |1
        if (bev2_top_y >= bev1_top_y) and (bev2_bot_y <= bev1_bot_y):
            #bev2 all inside bev1, TODO
            diff_u_left = bev2_left_x - bev1_left_x
            diff_u_right = bev1_right_x - bev2_right_x
            diff_u = max(diff_u_left, diff_u_right)
            diff_v_up = bev2_top_y - bev1_top_y
            diff_v_bot = bev1_bot_y - bev2_bot_y
            if diff_u <= diff_v_up and diff_u <= diff_v_bot: # move left / right
                if bbox1_info.pos_filter[1] < 0: #right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: #left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            else: # move down / up
                if bbox1_info.pos_filter[0] > 0: #up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else: #down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
        elif (bev2_top_y <= bev1_top_y) and (bev2_bot_y >= bev1_bot_y):
            #bev2 w inside bev1, bev1 h inside bev2, TODO
            diff_u_left = bev2_left_x - bev1_left_x
            diff_u_right = bev1_right_x - bev2_right_x
            diff_u = max(diff_u_left, diff_u_right)
            diff_v_up = bev2_top_y - bev1_top_y
            diff_v_bot = bev2_bot_y - bev1_bot_y
            if diff_u <= diff_v_up and diff_u <= diff_v_bot: # move left
                if bbox1_info.pos_filter[1] < 0: #right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: #left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            else: # move down / up
                if bbox1_info.pos_filter[0] > 0: #up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else: #down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
        elif (bev1_top_y <= bev2_top_y <= bev1_bot_y) and (bev2_bot_y >= bev1_bot_y):
            #bev2 move down, only one choice
            dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
            # print('===== move %d to down' % bbox2_info.tracking_id)
        elif (bev1_top_y <= bev2_bot_y <= bev1_bot_y) and (bev2_top_y <= bev1_top_y):
            #bev2 move up, only one choice
            dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
            # print('===== move %d to up' % bbox2_info.tracking_id)
    elif (bev2_left_x <= bev1_right_x <= bev2_right_x) and (bev2_left_x <= bev1_left_x <= bev2_right_x):
        # |2 |1 |1 |2
        if (bev1_top_y >= bev2_top_y) and (bev1_bot_y <= bev2_bot_y):
            #bev1 all inside bev2, TODO
            diff_u_left = bev1_left_x - bev2_left_x
            diff_u_right = bev2_right_x - bev1_right_x
            diff_u = max(diff_u_left, diff_u_right)
            diff_v_up = bev1_top_y - bev2_top_y
            diff_v_bot = bev2_bot_y - bev1_bot_y
            if diff_u <= diff_v_up and diff_u <= diff_v_bot: # move left
                if bbox1_info.pos_filter[1] < 0: #right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: #left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            else: # move down / up
                if bbox1_info.pos_filter[0] > 0: #up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else: #down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
        elif (bev1_top_y <= bev2_top_y) and (bev1_bot_y >= bev2_bot_y):
            #bev1 w inside bev2, bev2 h inside bev1, TODO
            diff_u_left = bev1_left_x - bev2_left_x
            diff_u_right = bev2_right_x - bev1_right_x
            diff_u = max(diff_u_left, diff_u_right)
            diff_v_up = bev2_top_y - bev1_top_y
            diff_v_bot = bev1_bot_y - bev2_bot_y
            if diff_u <= diff_v_up and diff_u <= diff_v_bot: # move left
                if bbox1_info.pos_filter[1] < 0: #right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: #left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            else: # move down / up
                if bbox1_info.pos_filter[0] > 0: #up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else: #down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
        elif (bev2_top_y <= bev1_bot_y <= bev2_bot_y) and (bev1_top_y <= bev2_top_y):
            #bev2 move down, only one choice
            dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
            # print('===== move %d to down' % bbox2_info.tracking_id)
        elif (bev2_top_y <= bev1_top_y <= bev2_bot_y) and (bev1_bot_y >= bev2_bot_y):
            #bev2 move up, only one choice
            dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
            # print('===== move %d to up' % bbox2_info.tracking_id)
    elif (bev2_left_x <= bev1_left_x <= bev2_right_x) and (bev1_right_x >= bev2_right_x):
        # |2 |1 |2 |1, move left or l-u / l-d
        if ((bev1_top_y <= bev2_top_y) and (bev1_bot_y >= bev2_bot_y)) or ((bev1_top_y >= bev2_top_y) and (bev1_bot_y <= bev2_bot_y)):
            # only one choice, bev2 move left
            dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            # print('===== move %d to left' % bbox2_info.tracking_id)
        elif (bev1_top_y <= bev2_bot_y <= bev1_bot_y) and (bev2_top_y <= bev1_top_y):
            # left-up
            tid = bbox2_info.tracking_id
            #history tracker
            if len(trackers[tid]) > 2: #long enough
                pos_3 = trackers[tid][-3]['info'].pos_filter
                pos_2 = trackers[tid][-2]['info'].pos_filter
                dw_32 = -(pos_3[1] - pos_2[1]); dh_32 = -(pos_3[0] - pos_2[0])
                slope = dh_32 / (dw_32+1e-6)
                print(slope, dh_32, dw_32)
                if diff_u <= diff_v and dw_32 > 0:#left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                elif dh_32 > 0: # up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else:
                    if abs(slope) > 1: #left
                        dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                    else:
                        dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
            else:
                if diff_u <= diff_v: # move left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                else: # move up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
        else:
            # left-down, (bev1_top_y <= bev2_top_y <= bev1_bot_y) and (bev2_bot_y >= bev1_bot_y)
            tid = bbox2_info.tracking_id
            if len(trackers[tid]) > 2: #long enough
                pos_3 = trackers[tid][-3]['info'].pos_filter
                pos_2 = trackers[tid][-2]['info'].pos_filter
                dw_32 = -(pos_3[1] - pos_2[1]); dh_32 = -(pos_3[0] - pos_2[0])
                slope = dh_32 / (dw_32+1e-6)
                print(slope, dh_32, dw_32)
                if diff_u <= diff_v and dw_32 > 0:#left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                elif dh_32 < 0:#down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
                else:
                    if abs(slope) > 1: #left
                        dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                    else:
                        dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
            else:
                if diff_u <= diff_v: # move left
                    dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                else: # move down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
    else:
        # |1 |2 |1 |2, move right or r-u / r-d
        if ((bev1_top_y <= bev2_top_y) and (bev1_bot_y >= bev2_bot_y)) or ((bev1_top_y >= bev2_top_y) and (bev1_bot_y <= bev2_bot_y)):
            # only one choice, bev2 move right 
            dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
            # print('===== move %d to right' % bbox2_info.tracking_id)
        elif (bev1_top_y <= bev2_bot_y <= bev1_bot_y) and (bev2_top_y <= bev1_top_y):
            # right-up
            tid = bbox2_info.tracking_id
            if len(trackers[tid]) > 2: #long enough
                pos_3 = trackers[tid][-3]['info'].pos_filter
                pos_2 = trackers[tid][-2]['info'].pos_filter
                dw_32 = -(pos_3[1] - pos_2[1]); dh_32 = -(pos_3[0] - pos_2[0])
                slope = dh_32 / (dw_32+1e-6)
                print(slope, dh_32, dw_32)
                if diff_u <= diff_v and dw_32 < 0:#right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                elif dh_32 > 0:#up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
                else:
                    if abs(slope) > 1: #right
                        dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                    else: #up
                        dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
            else:
                if diff_u <= diff_v: # move right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: # move up
                    dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
        else:
            # right-down
            tid = bbox2_info.tracking_id
            if len(trackers[tid]) > 2: #long enough
                pos_3 = trackers[tid][-3]['info'].pos_filter
                pos_2 = trackers[tid][-2]['info'].pos_filter
                dw_32 = -(pos_3[1] - pos_2[1]); dh_32 = -(pos_3[0] - pos_2[0])
                slope = dh_32 / (dw_32+1e-6)
                print(slope, dh_32, dw_32)
                if diff_u <= diff_v and dw_32 < 0:#right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                elif dh_32 < 0:#down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
                else:
                    if abs(slope) > 1: #right
                        dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                    else:
                        dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
            else:
                if diff_u <= diff_v: # move right
                    dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                else: # move down
                    dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
    new_bev_cent_u = bev2_cent_u + dw_pixel
    new_bev_cent_v = bev2_cent_v + dh_pixel
    new_dy = bev_range_w/2 - new_bev_cent_u * 0.1
    new_dx = bev_range_h/2 - new_bev_cent_v * 0.1
    new_pos_filter = [new_dx, new_dy]
    new_bbox2_info = BBox(cate=bbox2_info.cate, cate_index=bbox2_info.cate_index,
                        tracking_id=bbox2_info.tracking_id, tracking_age=bbox2_info.tracking_age,
                        uv_coord=bbox2_info.uv_coord, pos_filter=new_pos_filter,
                        obstacle_width=bbox2_info.obstacle_width, obstacle_length=bbox2_info.obstacle_length,
                        alpha=bbox2_info.alpha, heading_yaw=bbox2_info.heading_yaw)
    if dw_pixel != 0 or dh_pixel != 0:
        trackers[bbox2_info.tracking_id][-1]['info'] = new_bbox2_info, trackers
    return new_bbox2_info, trackers

def protect_each_car(bbox_infos:list, bev_range_config, trackers):
    bbox_infos.sort(key=sort_by_radiu)
    sort_bbox_infos = bbox_infos
    iou_thre = 0.0
    #two overlap
    new_list = []
    for ii in range(len(sort_bbox_infos)):
        bbox1_info, trackers = protect_ego_car(sort_bbox_infos[ii], bev_range_config, trackers)
        for jj in range(len(sort_bbox_infos[ii+1:])):
            bbox2_info = sort_bbox_infos[ii+1+jj]
            iou = bev_view_car_iou(bbox1_info, bbox2_info)
            if iou > iou_thre:
                print('----protect')
                print(bbox1_info.tracking_id, bbox2_info.tracking_id, iou)
                new_bbox2_info, trackers = finetune_two_bev(bbox1_info, bbox2_info, bev_range_config, trackers)
                sort_bbox_infos[ii+1+jj] = new_bbox2_info
        new_list.append(bbox1_info)
    return new_list

def smooth_bev_size(tracker_info_list:list):
    if len(tracker_info_list) < 4:
        return tracker_info_list
    bev_w_sizes = []
    bev_h_sizes = []
    for track_info in tracker_info_list:
        if track_info['missing_time'] == 0:
            bev_w_sizes.append(track_info['info'].obstacle_width)
            bev_h_sizes.append(track_info['info'].obstacle_length)
    if len(bev_h_sizes) > 0:
        weight_all, new_width, new_length = 0, 0, 0
        for ii, _ in enumerate(bev_w_sizes):
            weight = 1 + 0.1 * ii
            weight_all += weight
            new_width += (weight * bev_w_sizes[ii])
            new_length += (weight * bev_h_sizes[ii])
        new_width /= weight_all
        new_length /= weight_all
        last_bbox_info = tracker_info_list[-1]['info']
        new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                            tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                            uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                            obstacle_width=new_width, obstacle_length=new_length, 
                            alpha=last_bbox_info.alpha, heading_yaw=last_bbox_info.heading_yaw)
        tracker_info_list[-1]['info'] = new_bbox_info
    return tracker_info_list

def pi2degree(yaw):
    return yaw * 180 / 3.14

def degree2pi(degree):
    return degree * 3.14 / 180

def smooth_publish_track(tracker_info_list:list):
    if len(tracker_info_list) < 3:
        return tracker_info_list
    distance_dy_list = []
    yaw_list = []
    # dy, 横向距离平滑
    miss_num_list = []
    for tracker_info in tracker_info_list:
        distance_dy_list.append(tracker_info['info'].pos_filter[1])
        yaw_list.append(tracker_info['info'].heading_yaw)
        miss_num_list.append(tracker_info['missing_time'])
    max_miss_num = max(miss_num_list)
    smooth_distance_dy_list = []
    smooth_yaw_list = [] #btw, smooth yaw
    for i in range(len(distance_dy_list)): #0,1,2,3,4,5,6,7
        if i < 2:
            smooth_distance_dy_list.append(distance_dy_list[i])
            smooth_yaw_list.append(yaw_list[i])
        else:
            if max_miss_num > 0:
                smooth_y = distance_dy_list[i]
            else:
                smooth_y = 0.15 * smooth_distance_dy_list[i-2] + 0.25 * smooth_distance_dy_list[i-1] + 0.6 * distance_dy_list[i]
            if abs(abs(pi2degree(yaw_list[i]))-abs(pi2degree(yaw_list[i-1])))<5:
                if abs(abs(pi2degree(yaw_list[i]))-abs(pi2degree(yaw_list[i-2])))>160:
                    smooth_yaw = 0.3 * smooth_yaw_list[i-1] + 0.7 * yaw_list[i]
                else:
                    smooth_yaw = 0.15 * smooth_yaw_list[i-2] + 0.25 * smooth_yaw_list[i-1] + 0.6 * yaw_list[i]
            elif abs(abs(pi2degree(yaw_list[i]))-abs(pi2degree(yaw_list[i-1])))>160: #reverse
                # yaw_list[i] = degree2pi(yaw_list[i]/abs(yaw_list[i]) * (180 - abs(pi2degree(yaw_list[i]))))
                # smooth_yaw = 0.15 * smooth_yaw_list[i-2] + 0.25 * smooth_yaw_list[i-1] + 0.6 * degree2pi(yaw_list[i-1]/abs(yaw_list[i-1]) * (180 - abs(pi2degree(yaw_list[i]))))
                smooth_yaw = yaw_list[i]
            else:
                smooth_yaw = 0.05 * smooth_yaw_list[i-2] + 0.15 * smooth_yaw_list[i-1] + 0.8 * yaw_list[i]
            smooth_distance_dy_list.append(smooth_y)
            smooth_yaw_list.append(smooth_yaw)
    diff_dy = smooth_distance_dy_list[-1] - distance_dy_list[-1]
    last_bbox_info = tracker_info_list[-1]['info']
    # print('%d --- dy: %f' %(last_bbox_info.tracking_id, diff_dy))
    new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                            tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                            uv_coord=last_bbox_info.uv_coord, pos_filter=[last_bbox_info.pos_filter[0], smooth_distance_dy_list[-1]],
                            obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                            alpha=last_bbox_info.alpha, heading_yaw=smooth_yaw_list[-1])
    tracker_info_list[-1]['info'] = new_bbox_info
    # w = is_straight_car_by_track(tracker_info_list)
    # print('w', new_bbox_info.tracking_id, w)
    return tracker_info_list

def is_straight_car_by_rot(tracker_info_list:list):
    rot_angle_1 = tracker_info_list[-1]['info'].heading_yaw * 180 / 3.14
    rot_angle_2 = tracker_info_list[-2]['info'].heading_yaw * 180 / 3.14
    rot_angle_3 = tracker_info_list[-3]['info'].heading_yaw * 180 / 3.14
    diff1 = abs(rot_angle_1-rot_angle_2)
    diff2 = abs(rot_angle_3-rot_angle_2)
    return (abs(rot_angle_1)<5 or abs(abs(rot_angle_1)-180)<5) and (abs(rot_angle_2)<5 or abs(abs(rot_angle_2)-180)) and (abs(rot_angle_3)<5 or abs(abs(rot_angle_3)-180)) and diff1<5 and diff2<5

def is_straight_car_by_track(tracker_info_list:list):
    pos_1_h, pos_1_w = tracker_info_list[-1]['info'].pos_filter
    pos_2_h, pos_2_w = tracker_info_list[-2]['info'].pos_filter
    pos_3_h, pos_3_w = tracker_info_list[-3]['info'].pos_filter
    #最小二乘统计斜率，似乎不太行，垂直的斜率特别大, xy换位置
    x1, y1 = 0, 0
    y2, x2 = pos_2_w - pos_1_w, pos_2_h - pos_1_h
    y3, x3 = pos_3_w - pos_1_w, pos_3_h - pos_1_h
    mean_w = (x1 + x2 + x3) / 3
    sum_yx = y1 * (x1 - mean_w) + y2 * (x2 - mean_w) + y3 * (x3 - mean_w)
    sum_x2 = x1**2 + x2**2 + x3**2
    w = sum_yx / (sum_x2 - 3 * (mean_w**2))
    # sum_yx, sum_x2, sum_x = 0, 0, 0
    # m = len(tracker_info_list)
    # for i in range(m):
    #     y = tracker_info_list[i]['info'].pos_filter[1] - tracker_info_list[0]['info'].pos_filter[1]
    #     x = tracker_info_list[i]['info'].pos_filter[0] - tracker_info_list[0]['info'].pos_filter[0]
    #     sum_x += x
    # sum_x /= m
    # for i in range(m):
    #     y = tracker_info_list[i]['info'].pos_filter[1] - tracker_info_list[0]['info'].pos_filter[1]
    #     x = tracker_info_list[i]['info'].pos_filter[0] - tracker_info_list[0]['info'].pos_filter[0]
    #     sum_yx += y * (x - sum_x)
    #     sum_x2 += x ** 2
    # w = sum_yx / (sum_x2 - m * (sum_x **2))
    return abs(w) < 0.2

def smooth_straight_car_by_track(tracker_info_list:list):
    straight_range = (40, 10)
    cate = tracker_info_list[-1]['info'].cate_index
    if cate > 3:
        return tracker_info_list
    if len(tracker_info_list) < 3:
        return tracker_info_list
    position = tracker_info_list[-1]['info'].pos_filter
    if abs(position[0]) > straight_range[0] or abs(position[1]) > straight_range[1]:
        return tracker_info_list
    if is_straight_car_by_rot(tracker_info_list) and is_straight_car_by_track(tracker_info_list):
        if abs(pi2degree(tracker_info_list[-1]['info'].heading_yaw)) < 5:
            new_yaw = 0
        else:
            new_yaw = degree2pi(tracker_info_list[-1]['info'].heading_yaw/abs(tracker_info_list[-1]['info'].heading_yaw) * 179.9)
        last_bbox_info = tracker_info_list[-1]['info']
        new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                            tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                            uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                            obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length,
                            alpha=last_bbox_info.alpha, heading_yaw=new_yaw)
        tracker_info_list[-1]['info'] = new_bbox_info
    return tracker_info_list

def refine_slant_vehicle(publish_bbox_infos:list, trackers:dict):
    publish_cars_infos = []
    new_publish_bbox_infos = []
    for pinfo in publish_bbox_infos:
        if pinfo.cate_index <= 3:
            publish_cars_infos.append(pinfo)
        else:
            new_publish_bbox_infos.append(pinfo)
    publish_cars_infos.sort(key=sort_by_radiu)
    for publish_car in publish_cars_infos:
        tid = publish_car.tracking_id
        cur_degree = pi2degree(publish_car.heading_yaw)
        if abs(cur_degree) < 5 or abs(180 - abs(cur_degree) < 5) or len(trackers[tid]) < 2:
            new_publish_bbox_infos.append(publish_car)
        else:
            pos_h, pos_w = publish_car.pos_filter
            lane_cars = []
            for pcar in publish_cars_infos:
                if pcar.tracking_id == tid:
                    continue
                ppos_h, ppos_w = pcar.pos_filter
                if abs(pos_w - ppos_w) < 2 and abs(pos_h-ppos_h) < 30:
                    lane_cars.append(pcar)
            if len(lane_cars) >= 3:
                mean_degree = 0
                for lane_car in lane_cars:
                    ldegree = pi2degree(trackers[lane_car.tracking_id][-1]['info'].heading_yaw)
                    # ldegree = pi2degree(lane_car.heading_yaw)
                    caldegree = min(abs(ldegree), abs(180-abs(ldegree)))
                    mean_degree += caldegree
                mean_degree /= len(lane_cars)
                if mean_degree < 5 and abs(mean_degree - min(abs(cur_degree), abs(180-abs(cur_degree)))) > 8:
                    if abs(cur_degree) > 90:
                        dst_degree = cur_degree/abs(cur_degree) * (180- mean_degree)
                    else:
                        dst_degree = cur_degree/abs(cur_degree) * mean_degree
                    print('%d degree modify from %f to %f' % (tid, cur_degree, dst_degree))
                    dst_yaw = degree2pi(dst_degree)
                    new_bbox_info = BBox(cate=publish_car.cate, cate_index=publish_car.cate_index,
                                    tracking_id=publish_car.tracking_id, tracking_age=publish_car.tracking_age,
                                    uv_coord=publish_car.uv_coord, pos_filter=publish_car.pos_filter,
                                    obstacle_width=publish_car.obstacle_width, obstacle_length=publish_car.obstacle_length,
                                    alpha=publish_car.alpha, heading_yaw=dst_yaw)
                    new_publish_bbox_infos.append(new_bbox_info)
                    trackers[tid][-1]['info'] = new_bbox_info
                else:
                    new_publish_bbox_infos.append(publish_car)
            else:
                hori_cars = []
                for pcar in publish_cars_infos:
                    if pcar.tracking_id == tid:
                        continue
                    ppos_h, ppos_w = pcar.pos_filter
                    if abs(pos_h - ppos_h) < pcar.obstacle_length/2 and abs(pos_w-ppos_w)<8:
                        hori_cars.append(pcar)
                # if tid == 27:
                #     pdb.set_trace()
                if (len(hori_cars) + len(lane_cars)) >= 3 and len(hori_cars) > 0:
                    mean_hdegree = 0
                    for hori_car in hori_cars:
                        ldegree = pi2degree(trackers[hori_car.tracking_id][-1]['info'].heading_yaw)
                        caldegree = min(abs(ldegree), abs(180-abs(ldegree)))
                        mean_hdegree += caldegree
                    mean_hdegree /= len(hori_cars)
                    mean_degree = 0
                    for lane_car in lane_cars:
                        ldegree = pi2degree(trackers[lane_car.tracking_id][-1]['info'].heading_yaw)
                        caldegree = min(abs(ldegree), abs(180-abs(ldegree)))
                        mean_degree += caldegree
                    if len(lane_cars):
                        mean_degree /= len(lane_cars)
                        mean_degree = mean_degree * 0.6 + mean_hdegree * 0.4
                    else:
                        mean_degree = mean_hdegree
                    if mean_degree < 5 and abs(mean_degree - min(abs(cur_degree), abs(180-abs(cur_degree)))) > 8:
                        if abs(cur_degree) > 90:
                            dst_degree = cur_degree/abs(cur_degree) * (180- mean_degree)
                        else:
                            dst_degree = cur_degree/abs(cur_degree) * mean_degree
                        print('%d degree modify from %f to %f' % (tid, cur_degree, dst_degree))
                        dst_yaw = degree2pi(dst_degree)
                        new_bbox_info = BBox(cate=publish_car.cate, cate_index=publish_car.cate_index,
                                        tracking_id=publish_car.tracking_id, tracking_age=publish_car.tracking_age,
                                        uv_coord=publish_car.uv_coord, pos_filter=publish_car.pos_filter,
                                        obstacle_width=publish_car.obstacle_width, obstacle_length=publish_car.obstacle_length,
                                        alpha=publish_car.alpha, heading_yaw=dst_yaw)
                        new_publish_bbox_infos.append(new_bbox_info)
                        trackers[tid][-1]['info'] = new_bbox_info
                    else:
                        new_publish_bbox_infos.append(publish_car)
                else:
                    new_publish_bbox_infos.append(publish_car)
    return new_publish_bbox_infos, trackers

### smooth tracker ###