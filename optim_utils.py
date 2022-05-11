from copy import deepcopy
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
    ahead_range = 1.9
    dis_h, dis_w = position #[dx, dy]
    if dis_h > 0:
        if abs(dis_w) < ahead_range/2:
            show_flag = False
            print('%d Too ahead' % bbox_info.tracking_id)
    return show_flag

def is_ped_smooth(bbox_info):
    scale = 0.1
    bev_range_config = (60, 40)
    position = bbox_info.pos_filter
    bev_cent_u = (-position[1]+bev_range_config[1]/2) / scale
    bev_cent_v = (bev_range_config[0]/2 - position[0]) / scale
    if (0<bev_cent_u<bev_range_config[1]/scale and 0<bev_cent_v<bev_range_config[0]/scale):
        return True
    else:
        return False

def update_trackers(trackers:dict, fusion_bbox_infos, bev_range_config=(60, 40), scale=0.1):
    max_length_ = 8
    vanish_length_ = 8
    first_appear_ids = []
    for bbox_info in fusion_bbox_infos:
        bbox_show_flag = is_box_show(bbox_info, bev_range_config, scale) # show in bev range
        tracking_id = bbox_info.tracking_id

        if tracking_id not in trackers: # id first time appearance
            # TODO: 是否是大车id跳变导致的？是否需要修复id进行显示？否则会突然消失？还是将id跳变的tracker继承？ 前方已经做了这件事？
            # init tracker info
            first_appear_ids.append(tracking_id)
            track_info = {"info": bbox_info, "focus_time":1,
                          "missing_time":0, "match_flag":True,
                          "fill_flag":False, "smooth_cate": -1,
                          "link_flag":False}# "inhreit_id": -22}
            if not bbox_show_flag:
                track_info['focus_time'] = 0
                track_info['match_flag'] = False
            trackers[tracking_id] = [track_info]

        elif tracking_id in trackers:
            if len(trackers[tracking_id]) < 3: #
                first_appear_ids.append(tracking_id)
            track_info = deepcopy(trackers[tracking_id][-1])
            bbox_info_new = BBox(cate=bbox_info.cate, cate_index=bbox_info.cate_index,
                                        tracking_id=bbox_info.tracking_id, tracking_age=track_info['info'].tracking_age+1,
                                        uv_coord=bbox_info.uv_coord, pos_filter=bbox_info.pos_filter,
                                        obstacle_width=bbox_info.obstacle_width, obstacle_length=bbox_info.obstacle_length)
            track_info["info"] = bbox_info_new
            track_info['fill_flag'] = False
            track_info['smooth_cate'] = -1
            track_info['link_flag'] = False
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

    #first appear check
    for first_id in first_appear_ids:
        first_tracker = trackers[first_id][-1]
        if first_tracker['focus_time'] == 0:
            continue
        first_cate = first_tracker['info'].cate_index
        if first_cate > 3: # only for car
            continue
        for tid, tracker_info in trackers.items():
            if tid in first_appear_ids:
                continue
            if len(tracker_info) < max_length_:
                continue
            cur_cate = tracker_info[-1]['info'].cate_index
            cur_focus_time = tracker_info[-2]['focus_time']
            if first_cate == cur_cate and cur_focus_time > max_length_:
                inter_scale, diff_w_scale = bev_view_overlap(first_tracker['info'], tracker_info[-1]['info'])
                if inter_scale > 0:
                    print('fisrt id: %d , overlap id: %d, overlap_scale: %f, diff_scale: %f' % (first_id, tid, inter_scale, diff_w_scale))
                if inter_scale > 0.2 and diff_w_scale < 0.4:
                    print('%d maybe redetected in %d' % (first_id, tid))
                    if tracker_info[-1]['missing_time'] == 0:
                        trackers[first_id][-1]["missing_time"] += 1 #前面的没有漏检，初次检测到的重复框记miss
                    else:
                        print('%d replace %d' % (first_id, tid)) #前面的漏检了，当前检测到的继承它的focus信息，并且将之前的从tracker中删除
                        trackers[first_id][-1]["focus_time"] = tracker_info[-2]['focus_time'] + 1
                        trackers[first_id][-1]["missing_time"] = 0
                        # print(trackers[first_id][-1])
                        trackers[tid][-1]['missing_time'] = vanish_length_+1

    del_list = []
    for tracking_id, track_info in trackers.items():
        if trackers[tracking_id][-1]['missing_time'] > vanish_length_:
            del_list.append(tracking_id)
    for tracking_id in del_list:
        if tracking_id in trackers:
            del trackers[tracking_id]

    # for tracking_id, tracker_infos in trackers.items():
    #     if tracking_id == 83:
    #             print('----')
    #             for item in tracker_infos:
    #                 print(item)
    return trackers

### update view trackers ###

### filter bev car ###
def delay_n_filter(tracker_infos:list):
    """
    filter main function, add mothed in this function
    """
    delay_n = 1
    # delay_n = 0
    if tracker_infos[-1]['info'].cate_index > 3: # not car, delay more
        delay_n += 1
    if tracker_infos[-1]['info'].pos_filter[1] > 12: #
        delay_n += 1
    middle_range = (15, 5) # h:15m, w:5m

    publish_flag = False
    last_tracker_info = tracker_infos[-1]
    tracking_id = tracker_infos[-1]['info'].tracking_id

    #TODO: more condition
    # too big bbox suddenly appear
    # if last_tracker_info['info'].uv_coord[2] * last_tracker_info['info'].uv_coord[3] > 0.6*512*216:
    #     delay_n -= 1

    # ped delay_n +=1
    if last_tracker_info['info'].cate_index > 3:
        delay_n += 1

    # 1. simple focus and missing times
    if last_tracker_info['focus_time'] < (1+delay_n): #first appearence
        publish_flag = False
    elif last_tracker_info['missing_time'] > 0: # miss in this frame
        publish_flag = False
    else:
        publish_flag = True

    # 2. burst in the middle view
    burst_delay_n = is_burst_in_middle_view(tracker_infos, delay_n)
    if publish_flag:
        if burst_delay_n > delay_n:
            print('tid: ', tracking_id, 'burst in!')
            #TODO: left view, burst if the opposite headstock?
            if last_tracker_info['focus_time'] < (1+burst_delay_n): #first appearence
                publish_flag = False
            elif last_tracker_info['missing_time'] > 0: # miss in this frame
                publish_flag = False
            else:
                publish_flag = True

    if publish_flag:
        small_delay_n = is_not_middle_and_small(tracker_infos, burst_delay_n)
        if small_delay_n > burst_delay_n:
            print('tid:', tracking_id, 'far and bbox not big!')
            if last_tracker_info['focus_time'] < (1+small_delay_n):
                publish_flag = False
            elif last_tracker_info['missing_time'] > 0:
                publish_flag = False
            else:
                publish_flag = True
    else:
        small_delay_n = burst_delay_n

    # 3.the more time flicker, the more frame delay
    if publish_flag:
        flicker_delay_n = is_flicker(tracker_infos, small_delay_n, middle_range)
        if flicker_delay_n != small_delay_n:
            print('tid: ', tracking_id, 'flicker delay more: ', flicker_delay_n)
            if last_tracker_info['focus_time'] < (1+flicker_delay_n):
                publish_flag = False
            elif last_tracker_info['missing_time'] > 0:
                publish_flag = False
            else:
                publish_flag = True
    else:
        flicker_delay_n = small_delay_n
        if last_tracker_info['focus_time'] < (1+flicker_delay_n):
            publish_flag = False
        elif last_tracker_info['missing_time'] > 0:
            publish_flag = False
        else:
            publish_flag = True

    return publish_flag

def is_flicker(tracker_infos:list, delay_n=1, middle_range=(15, 5)):
    new_delay_n = delay_n
    tracking_id = tracker_infos[-1]['info'].tracking_id
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
            # focus_value = focus_list[ii]
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

def is_burst_in_middle_view(tracker_infos:list, delay_n=1):
    #only for vehical
    middle_range = (18, 5)
    cate_index = tracker_infos[-1]['info'].cate_index
    tracking_id = tracker_infos[-1]['info'].tracking_id
    if cate_index > 3:
        return delay_n #ignore not car
    else:
        burst_delay_n = delay_n
        pos_h, pos_w = tracker_infos[-1]['info'].pos_filter #pos_x ->zongxinag  pos_y->hengxiang
        coord = tracker_infos[-1]['info'].uv_coord
        leftx = coord[0]
        right_x = leftx + coord[2]
        if abs(pos_h) < middle_range[0]: #h inside 18m
            if tracker_infos[-1]['focus_time'] > (delay_n+2) or abs(pos_w) < middle_range[1]:#or coord[2]*coord[3] >= size_range
                return burst_delay_n #apperence more than delay_n, or too close to left/right side
            elif abs(right_x - 512) < 10:
                return burst_delay_n
            else:
                burst_delay_n += 1
                return burst_delay_n #else is suddenly appearence
        else:
            return burst_delay_n

def is_not_middle_and_small(tracker_infos:list, delay_n=1):
    #only for vehical
    middle_range = (20, 5)
    cate_index = tracker_infos[-1]['info'].cate_index
    tracking_id = tracker_infos[-1]['info'].tracking_id
    if cate_index > 3:
        return delay_n #ignore not car
    else:
        tid = tracker_infos[-1]['info'].tracking_id
        # if tid == 42:
        #     pdb.set_trace()
        small_delay_n = delay_n
        pos_h, pos_w = tracker_infos[-1]['info'].pos_filter #pos_x ->zongxiang  pos_y->hengxiang
        coord = tracker_infos[-1]['info'].uv_coord
        leftx = coord[0]
        right_x = leftx + coord[2]
        width, height = coord[2:]
        if abs(pos_h) > middle_range[0] and pos_h < 0 and width * height < (50*50): #h outside 20m
            if tracker_infos[-1]['focus_time'] > (delay_n+2):
                return small_delay_n #apperence more than delay_n
            elif len(tracker_infos) < 6: #边缘突然出现
                if (height / width) > 2 and (abs(leftx) < 10 or abs(right_x - 512) < 10):
                    small_delay_n += 2
                else:
                    small_delay_n += 1
                return small_delay_n #else is suddenly appearence
            else:
                return small_delay_n
        else:
            return small_delay_n

### filter bev car ###

### smooth tracker ###
def is_left_truck(tracker_info:BBox):
    cate_index = tracker_info.cate_index
    cate_flag = (cate_index == 2) #truck : 2-1=1 bus: 3-1=2
    posy = tracker_info.pos_filter[1] #pos_y, wpixel
    pos_flag = (posy > 0)
    return cate_flag and pos_flag

def pair_publish_left_truck(truck_trackers:list, trackers:dict):
    left_front_tail_list = [] # left_x close to 0
    left_rear_head_list = [] # right_c close to 512
    pair_list = []
    for truck in truck_trackers:
        tracking_id = truck.tracking_id
        coord = truck.uv_coord
        left_x = coord[0]
        right_x = left_x + coord[2]
        pos_x = truck.pos_filter[0]
        if pos_x > 0 and abs(left_x) < 5:
            left_front_tail_list.append(truck)
        if pos_x < 0 and abs(right_x - 512) < 10:
            left_rear_head_list.append(truck)
    if len(left_front_tail_list) and len(left_rear_head_list):
        tail_pair_flag = [False] * len(left_front_tail_list)
        head_pair_flag = [False] * len(left_rear_head_list)
        for ii, tail in enumerate(left_front_tail_list):
            for jj, head in enumerate(left_rear_head_list):
                pos_y_tail = tail.pos_filter[1]
                pos_y_head = head.pos_filter[1]
                tail_top_y = tail.uv_coord[1]
                tail_bot_y = tail_top_y + tail.uv_coord[3]
                head_top_y = head.uv_coord[1]
                head_bot_y = head_top_y + head.uv_coord[3]
                pos_flag = (abs(pos_y_head - pos_y_tail) < 1.5)
                ###can't handle pos_y diff is too big
                # inside_flag = (tail_top_y > head_top_y) and (tail_bot_y < head_bot_y)
                inside_flag = (tail_top_y > head_top_y)
                if pos_flag and inside_flag and (not tail_pair_flag[ii]) and (not head_pair_flag[jj]):
                    pair_list.append([{'id':tail.tracking_id, 'truck':tail}, {'id':head.tracking_id, 'truck':head}])
                    tail_pair_flag[ii] = True
                    head_pair_flag[jj] = True
    all_pair_id_list = []
    if len(pair_list):
        for item in pair_list:
            all_pair_id_list += [item[0]['id'], item[1]['id']]
        not_pair_truck = []
        for truck in truck_trackers:
            tracking_id = truck.tracking_id
            if tracking_id not in all_pair_id_list:
                not_pair_truck.append(truck)
        pair_truck = []
        for pair in pair_list:
            tail = pair[0]['truck']
            head = pair[1]['truck']
            #TODO: new_truck_length
            new_truck_info = BBox(cate=head.cate, cate_index=head.cate_index,
                                        tracking_id=head.tracking_id, tracking_age=tail.tracking_age+1,
                                        uv_coord=head.uv_coord, pos_filter=head.pos_filter,
                                        obstacle_width=head.obstacle_width, obstacle_length=head.obstacle_length)
            trackers[head.tracking_id][-1]['info'] = new_truck_info
            trackers[head.tracking_id][-1]['focus_time'] = trackers[tail.tracking_id][-1]['focus_time'] + 1
            trackers[head.tracking_id][-1]['missing_time'] = trackers[head.tracking_id][-1]['missing_time']
            trackers[head.tracking_id][-1]['match_flag'] = False
            trackers[head.tracking_id][-1]['fill_flag'] = trackers[head.tracking_id][-1]['fill_flag']
            trackers[head.tracking_id][-1]['smooth_cate'] = -1
            trackers[head.tracking_id][-1]['link_flag'] = True

            trackers[tail.tracking_id][-1]['info'] = tail
            trackers[tail.tracking_id][-1]['focus_time'] = 0
            trackers[tail.tracking_id][-1]['missing_time'] = trackers[tail.tracking_id][-1]['missing_time'] +1
            trackers[tail.tracking_id][-1]['match_flag'] = False
            trackers[tail.tracking_id][-1]['fill_flag'] = trackers[tail.tracking_id][-1]['fill_flag']
            trackers[tail.tracking_id][-1]['smooth_cate'] = -1
            trackers[tail.tracking_id][-1]['link_flag'] = True
            pair_truck.append(new_truck_info)
        new_truck_list = not_pair_truck+pair_truck
        print('!!!!!!!')
        print(all_pair_id_list)
    else:
        new_truck_list = truck_trackers

    return new_truck_list, trackers

def bev_view_car_iou(bev1_info, bev2_info, bev_range_config=(60, 40), scale=0.1):
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
    union_area = bev1_box_area + bev2_box_area - inter_area
    iou = inter_area / union_area

    return iou

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
    max_missing_num = int(max_length_ / 2)
    # fill_range=(20, 12)
    fill_range=(30, 18)
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
    if tracker[-1]['info'].uv_coord[2] > 512/2: #w > 256
        max_length_ = 6 ###test for id69

    last_bbox_info = tracker[-1]['info']
    # uvcoord = last_bbox_info.uv_coord
    # height = uvcoord[3]
    # width = uvcoord[2]
    # lefx_x = uvcoord[0]
    # right_x = uvcoord[0] + uvcoord[2]
    # if (abs(lefx_x) < 10 or abs(right_x - 512) < 10) and height < 512/7 and height/width>1.5:
    #     print('%d too close to edge, dont fill' % last_bbox_info.tracking_id)
    #     return tracker, trackers

    if len(tracker) >= max_length_ and tracker[-1]['missing_time'] > 0:
        position = last_bbox_info.pos_filter #pos_x zongxinag , pos_y hengxiang
        position_last = tracker[-2]['info'].pos_filter
        new_pos_x = position[0] + (position[0] - position_last[0]) / tracker[-1]['missing_time']
        new_pos_y = position[1] + (position[1] - position_last[1]) / tracker[-1]['missing_time']
        new_position = [new_pos_x, new_pos_y]
        new_last_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)

    if (tracking_age>=max_length_) and (max_focus_time >= max_length_) and (0< tracker[-1]['missing_time'] < max_missing_num):
        position = last_bbox_info.pos_filter #pos_x zongxinag , pos_y hengxiang
        position_last = tracker[-2]['info'].pos_filter
        new_pos_x = position[0] + (position[0] - position_last[0]) / tracker[-1]['missing_time']
        new_pos_y = position[1] + (position[1] - position_last[1]) / tracker[-1]['missing_time']
        new_position = [new_pos_x, new_pos_y]
        new_last_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
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
            # if tracking_id == 3:
            #     pdb.set_trace()

            if max_overlap > 0.2 and dst_diff_w_scale < 0.2: #and replace_bbox_info.tracking_age < max_length_: (age不准确，不能反应目标的出现次数，publish list 中的信息没有维护出现次数)
                pass
            else:
                overlap_id = -22
            if overlap_id > 0: #overlap / id switch or occlusion
                print('%d replace %d, stop filling' % (overlap_id, tracking_id))
                tracker[-1]['fill_flag'] = False
                ### tracker[-1]['missing_time'] = max_length_-1
                # for rid, item in trackers.items():
                #     if rid == overlap_id:
                #         trackers[rid][-1]['focus_time'] = tracker[-2]['focus_time'] + 1

            else:
                fill_flag = True
                new_bbox_track = {}
                new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                        tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age+1,
                                        uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                        obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
                new_bbox_track['info'] = new_bbox_info
                new_bbox_track['focus_time'] = tracker[-2]['focus_time'] + 1
                new_bbox_track['missing_time'] = tracker[-1]['missing_time']
                new_bbox_track['match_flag'] = False
                new_bbox_track['smooth_cate'] = -1
                new_bbox_track['link_flag'] = False
                tracker[-1] = new_bbox_track
                new_bbox_track['fill_flag'] = fill_flag

        else: # out of filling range
            new_bbox_track = {}
            new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                    tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                    uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                    obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
            new_bbox_track['info'] = new_bbox_info
            new_bbox_track['focus_time'] = tracker[-1]['focus_time']
            new_bbox_track['missing_time'] = tracker[-1]['missing_time']
            new_bbox_track['match_flag'] = False
            new_bbox_track['smooth_cate'] = -1
            new_bbox_track['link_flag'] = False
            tracker[-1] = new_bbox_track
            tracker[-1]['fill_flag'] = fill_flag #false

    else: # don't meet filling conditions
        tracker[-1]['fill_flag'] = fill_flag #false
        if len(tracker) >= max_length_ and tracker[-1]['missing_time'] > 0:
            new_bbox_track = {}
            new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                    tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                                    uv_coord=last_bbox_info.uv_coord, pos_filter=new_position,
                                    obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
            new_bbox_track['info'] = new_bbox_info
            new_bbox_track['focus_time'] = tracker[-1]['focus_time']
            new_bbox_track['missing_time'] = tracker[-1]['missing_time']
            new_bbox_track['match_flag'] = False
            new_bbox_track['smooth_cate'] = -1
            new_bbox_track['link_flag'] = False
            new_bbox_track['fill_flag'] = fill_flag #false
            tracker[-1] = new_bbox_track
    return tracker, trackers

def link_missing_ped(ped_trackers:list):
    disappear_ped_list = []
    suddenly_appear_ped_list = []
    link_to_publish_ped_infos = []
    ped_ids = []
    for ped_tracker in ped_trackers:
        tracking_id = ped_tracker[-1]['info'].tracking_id
        ped_ids.append(tracking_id)
        if ped_tracker[-1]['info'].tracking_age > 4 and ped_tracker[-1]['missing_time'] == 1: #suddenly disappear
            disappear_ped_list.append(ped_tracker)
        # elif (ped_tracker[-1]['info'].tracking_age < 3) and (ped_tracker[-1]['missing_time'] == 0):
        elif (len(ped_tracker) < 3) and (ped_tracker[-1]['missing_time'] == 0) and (not ped_tracker[-1]['link_flag']):
            suddenly_appear_ped_list.append(ped_tracker)
    # if 56 in ped_ids:
    #     pdb.set_trace()
    for dis_ped_tracker in disappear_ped_list:
        # if dis_ped_tracker[-1]['info'].tracking_id == 212:
        #     pdb.set_trace()
        position_now_y = dis_ped_tracker[-1]['info'].pos_filter[1]
        dis_ped_height = dis_ped_tracker[-1]['info'].uv_coord[3]
        link_id = -1
        min_height_scale = 1000
        for suddenly_ped_tracker in suddenly_appear_ped_list:
            sudden_ped_height = suddenly_ped_tracker[-1]['info'].uv_coord[3]
            height_scale = min(abs(sudden_ped_height-dis_ped_height) / dis_ped_height, abs(sudden_ped_height-dis_ped_height) / sudden_ped_height)
            if (height_scale < 0.4):
                if height_scale < min_height_scale:
                    min_height_scale = height_scale
                    link_id = suddenly_ped_tracker[-1]['info'].tracking_id

        if link_id != -1:
            for suddenly_ped_tracker in suddenly_appear_ped_list:
                if suddenly_ped_tracker[-1]['info'].tracking_id == link_id:
                    sudden_pos_y = suddenly_ped_tracker[-1]['info'].pos_filter[1]
                    if (not suddenly_ped_tracker[-1]['link_flag']) and (sudden_pos_y*position_now_y)>0:
                        last_bbox_info = suddenly_ped_tracker[-1]['info']
                        print('------- link %d with %d ------' % (dis_ped_tracker[-1]['info'].tracking_id, link_id))
                        # if (link_id == 60):
                        #     pdb.set_trace()
                        new_bbox_track = {}
                        new_bbox_info = BBox(cate=last_bbox_info.cate, cate_index=last_bbox_info.cate_index,
                                                tracking_id=last_bbox_info.tracking_id, tracking_age=dis_ped_tracker[-1]['info'].tracking_age+1,
                                                uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                                                obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
                        new_bbox_track['info'] = new_bbox_info
                        if len(dis_ped_tracker) == 1: #id switch too fast?
                            new_bbox_track['focus_time'] = dis_ped_tracker[-1]['focus_time'] + 1
                        else:
                            new_bbox_track['focus_time'] = dis_ped_tracker[-2]['focus_time'] + 1
                        new_bbox_track['missing_time'] = 0
                        new_bbox_track['match_flag'] = False
                        new_bbox_track['fill_flag'] = dis_ped_tracker[-1]['fill_flag']
                        new_bbox_track['smooth_cate'] = -1
                        new_bbox_track['link_flag'] = True
                        suddenly_ped_tracker[-1] = new_bbox_track
                        link_to_publish_ped_infos.append(suddenly_ped_tracker)

    return link_to_publish_ped_infos

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

def cate_is_main_cate(cate_list, cate):
    cate0 = cate_list[0]
    if cate0 > 3:
        main_cate = 1
    else:
        main_cate = 0
    if cate > 3:
        cur_cate = 1
    else:
        cur_cate = 0
    return cur_cate == main_cate

def vote_cate(tracker:list, vote_lenth=5):
    if len(tracker) < vote_lenth: # nead enough infomation
        return tracker
    if tracker[-1]['missing_time'] > 0:
        return tracker
    tracking_id = tracker[-1]['info'].tracking_id
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
    max_time_cate = -1
    max_cate_time = -1
    for cate in cate_num:
        if cate_num[cate] > max_cate_weight:
            max_cate = cate
            max_cate_weight = cate_num[cate]

    main_cate = refine_cate_list[0]
    if (not same_flag or not rough_same_flag):
        if max_cate != main_cate:
            if cate_time[main_cate] > 2:
                max_cate = main_cate
    # if tracker[-1]['info'].tracking_id == 163 and (not same_flag or not rough_same_flag) and max_cate==2:
    #     pdb.set_trace()
    # if max_cate != cate_list[-1] and (not same_flag or not rough_same_flag):
    #     max_cate = max_time_cate

    # if tracker[-1]['info'].tracking_id == 163 and not same_flag:
    #     pdb.set_trace()
    if max_cate != tracker[-1]['info'].cate_index:
        print('~~~~~~~~~~~~~')
        # if tracker[-1]['info'].tracking_id == 163:
        #     pdb.set_trace()
        print('tid: ', tracker[-1]['info'].tracking_id, 'cate switch: ', tracker[-1]['info'].cate_index, max_cate)
        for c in cate_num:
            print(c, cate_num[c])
        new_cate = index2cate(max_cate-1)
        last_bbox_info = tracker[-1]['info']
        new_bbox_track = {}
        new_bbox_info = BBox(cate=new_cate, cate_index=max_cate,
                            tracking_id=last_bbox_info.tracking_id, tracking_age=last_bbox_info.tracking_age,
                            uv_coord=last_bbox_info.uv_coord, pos_filter=last_bbox_info.pos_filter,
                            obstacle_width=last_bbox_info.obstacle_width, obstacle_length=last_bbox_info.obstacle_length)
        new_bbox_track['info'] = new_bbox_info
        new_bbox_track['focus_time'] = tracker[-1]['focus_time']
        new_bbox_track['missing_time'] = tracker[-1]['missing_time']
        new_bbox_track['match_flag'] = tracker[-1]['match_flag']
        new_bbox_track['fill_flag'] = tracker[-1]['fill_flag']
        new_bbox_track['link_flag'] = tracker[-1]['link_flag']
        new_bbox_track['smooth_cate'] = last_bbox_info.cate_index
        tracker[-1] = new_bbox_track #replace the newest infomation
    return tracker

def protect_ego_car(bbox_info, bev_range_config):
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
                        obstacle_width=bbox_info.obstacle_width, obstacle_length=bbox_info.obstacle_length)
    return new_bbox_info

def sort_by_distx(bbox_infos):
    return abs(bbox_infos.pos_filter[1]) # x dist from ego car

def sort_by_disty(bbox_infos):
    return abs(bbox_infos.pos_filter[0]) # y dist from ego car

def sort_by_distarrond(bbox_infos):
    return min(abs(bbox_infos.pos_filter[0]), abs(bbox_infos.pos_filter[1]))

def sort_by_radiu(bbox_infos):
    return 1/bbox_infos.obstacle_length*bbox_infos.pos_filter[0]*bbox_infos.pos_filter[0] +bbox_infos.pos_filter[1]*bbox_infos.pos_filter[1]

def finetune_two_bev(bbox1_info, bbox2_info, bev_range_config): # 'x' / 'y'
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
    # if (bbox1_info.tracking_id==2) and (bbox2_info.tracking_id==7):
    #     pdb.set_trace()

    bev1_cent_u = (-bbox1_info.pos_filter[1]+bev_range_w/2) / 0.1; bev1_cent_v = (bev_range_h/2 - bbox1_info.pos_filter[0]) / 0.1
    bev1_left_x = bev1_cent_u - bev1_box_w / 2; bev1_right_x = bev1_cent_u + bev1_box_w / 2
    bev1_top_y = bev1_cent_v - bev1_box_l / 2; bev1_bot_y = bev1_cent_v + bev1_box_l / 2

    bev2_cent_u = (-bbox2_info.pos_filter[1]+bev_range_w/2) / 0.1; bev2_cent_v = (bev_range_h/2 - bbox2_info.pos_filter[0]) / 0.1
    bev2_left_x = bev2_cent_u - bev2_box_w / 2; bev2_right_x = bev2_cent_u + bev2_box_w / 2
    bev2_top_y = bev2_cent_v - bev2_box_l / 2; bev2_bot_y = bev2_cent_v + bev2_box_l / 2

    diff_u = min(bev1_right_x, bev2_right_x) - max(bev1_left_x, bev2_left_x)
    diff_v = min(bev1_bot_y, bev2_bot_y) - max(bev1_top_y, bev2_top_y)

    dw_pixel = 0; dh_pixel = 0; protect_pixel_w = 5; protect_pixel_h = 5
    if diff_u < diff_v: # move to right / left
        if (bev1_left_x <= bev2_right_x <= bev1_right_x) and (bev1_left_x <= bev2_left_x <= bev1_right_x):
            if abs(bbox1_info.pos_filter[1]) > abs(bbox2_info.pos_filter[1]): #？what
                dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
                # print('===== move %d to right' % bbox2_info.tracking_id)
            else:
                dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
                # print('===== move %d to left' % bbox2_info.tracking_id)
        elif (bev2_left_x <= bev1_right_x <= bev2_right_x) and (bev2_left_x <= bev1_left_x <= bev2_right_x):
            pass #TODO: need more cases. 146,147->truck boom to 2
        elif bev1_left_x <= bev2_right_x <= bev1_right_x:
            dw_pixel = - (bev2_right_x - bev1_left_x + protect_pixel_w)
            # print('===== move %d to left' % bbox2_info.tracking_id)
        elif bev1_left_x <= bev2_left_x <= bev1_right_x:
            dw_pixel = bev1_right_x - bev2_left_x + protect_pixel_w
            # print('===== move %d to right' % bbox2_info.tracking_id)
    else:
        if bev1_top_y <= bev2_top_y <= bev1_bot_y:
            dh_pixel = bev1_bot_y - bev2_top_y + protect_pixel_h
            # print('===== move %d to down' % bbox2_info.tracking_id)
        elif bev1_top_y <= bev2_bot_y <= bev1_bot_y:
            dh_pixel = - (bev2_bot_y - bev1_top_y + protect_pixel_h)
            # print('===== move %d to up' % bbox2_info.tracking_id)
    new_bev_cent_u = bev2_cent_u + dw_pixel
    new_bev_cent_v = bev2_cent_v + dh_pixel
    new_dy = bev_range_w/2 - new_bev_cent_u * 0.1
    new_dx = bev_range_h/2 - new_bev_cent_v * 0.1
    new_pos_filter = [new_dx, new_dy]
    new_bbox2_info = BBox(cate=bbox2_info.cate, cate_index=bbox2_info.cate_index,
                        tracking_id=bbox2_info.tracking_id, tracking_age=bbox2_info.tracking_age,
                        uv_coord=bbox2_info.uv_coord, pos_filter=new_pos_filter,
                        obstacle_width=bbox2_info.obstacle_width, obstacle_length=bbox2_info.obstacle_length)
    return new_bbox2_info

def protect_each_car(bbox_infos:list, bev_range_config):
    veh_bbox_infos = []
    ped_bbox_infos = []
    for bbox_info in bbox_infos:
        if bbox_info.cate_index > 3:
            ped_bbox_infos.append(bbox_info)
        else:
            veh_bbox_infos.append(bbox_info)
    veh_bbox_infos.sort(key=sort_by_radiu)
    ped_bbox_infos.sort(key=sort_by_radiu)
    sort_bbox_infos = veh_bbox_infos + ped_bbox_infos
    iou_thre = 0.0
    #two overlap
    new_list = []
    for ii in range(len(sort_bbox_infos)):
        bbox1_info = protect_ego_car(sort_bbox_infos[ii], bev_range_config)
        for jj in range(len(sort_bbox_infos[ii+1:])):
            bbox2_info = sort_bbox_infos[ii+1+jj]
            iou = bev_view_car_iou(bbox1_info, bbox2_info)
            if iou > iou_thre:
                # print(iou)
                print(bbox1_info.tracking_id, bbox2_info.tracking_id, iou)
                new_bbox2_info = finetune_two_bev(bbox1_info, bbox2_info, bev_range_config)
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
                            obstacle_width=new_width, obstacle_length=new_length)
        tracker_info_list[-1]['info'] = new_bbox_info
    return tracker_info_list

def straight_publish_track(tracker_info_list:list):
    distance_dy_list = []
    # dy, 横向距离平滑
    for tracker_info in tracker_info_list:
        pass

### smooth tracker ###