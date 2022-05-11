import os
import os.path as osp
import cv2
import argparse

from replay_utils import *
from optim_utils import *

def parse_args():
    parser = argparse.ArgumentParser(description='Replay or optimization detection result for SQ')
    parser.add_argument('--res', default='./', help='superior folder of result dir and image dir')
    parser.add_argument('--save', action='store_true', help='whether save results, store_true')
    parser.add_argument('--dst', required=False, default='./draw_result', help='result save dir')
    parser.add_argument('--tid', default=[], required=False, nargs='+', type=int)
    parser.add_argument('--start', default=0, required=False, type=int)
    args = parser.parse_args()
    return args

def sort_rule(name):
    return int(name.split('.')[0])

def filter_publish_bbox_infos(trackers:dict, bev_range_config):
    publish_bbox_infos = []
    miss_tracker_infos = []
    miss_ped_tracker_infos = []

    # actually publish bev
    for tracking_id, tracker_infos in trackers.items():
        tracker_infos = vote_cate(tracker_infos)
        tracker_infos = smooth_bev_size(tracker_infos)
        if delay_n_filter(tracker_infos):
            publish_bbox_infos.append(tracker_infos[-1]['info'])
        else:
            if tracker_infos[-1]['info'].cate_index > 3 and is_ped_smooth(tracker_infos[-1]['info']): # not car, is ped
                miss_ped_tracker_infos.append(tracker_infos)
            else:
                miss_tracker_infos.append(tracker_infos)

    # link some id swith pedestrian and so on
    link_bbox_infos = []
    link_bbox_ids = []
    link_to_publish_ped_infos = link_missing_ped(miss_ped_tracker_infos)
    for link_ped in link_to_publish_ped_infos:
        tracking_id = link_ped[-1]['info'].tracking_id
        trackers[tracking_id] = link_ped
        link_bbox_infos.append(link_ped[-1]['info'])
        link_bbox_ids.append(tracking_id)
    for item in miss_ped_tracker_infos:
        tracking_id = item[-1]['info'].tracking_id
        if tracking_id not in link_bbox_ids:
            last_ped_info = item[-1]
            if item[-1]['link_flag'] and last_ped_info['missing_time'] == 0 and last_ped_info['focus_time'] > 0 and last_ped_info['info'].tracking_age > last_ped_info['focus_time']:
                # id switch too fast, link 1 frame than swicth again
                link_bbox_infos.append(last_ped_info['info'])
    publish_bbox_infos += link_bbox_infos

    # fill some strong exist but missing bev
    fill_bbox_infos = []
    for missing_tracker in miss_tracker_infos:
        # 1. real miss detect;TODO: 2. id switch + delay, not show; 3. 1 big truck ->boom to 2
        missing_tracker, trackers = fill_missing_car(missing_tracker, publish_bbox_infos, trackers)
        if missing_tracker[-1]['fill_flag']:
            # if missing_tracker[-1]['info'].tracking_id == 2:
            #     pdb.set_trace()
            fill_bbox_infos.append(missing_tracker[-1]['info'])
            print('========fill========', missing_tracker[-1]['info'].tracking_id)
    publish_bbox_infos += fill_bbox_infos

    
    # truck_list = []
    # not_truck_list = []
    # for item in publish_bbox_infos:
    #     if is_left_truck(item):
    #         truck_list.append(item)
    #     else:
    #         not_truck_list.append(item)
    # new_truck_list, trackers = pair_publish_left_truck(truck_list, trackers)
    # publish_bbox_infos = new_truck_list + not_truck_list

    publish_bbox_infos = protect_each_car(publish_bbox_infos, bev_range_config)
    return publish_bbox_infos, trackers

if __name__ == '__main__':
    args = parse_args()

    resroot = args.res
    save_flag = args.save
    dst_dir = args.dst
    dst_tracking_id_list = args.tid
    file_index = args.start

    left_front_imgdir = osp.join(resroot, 'left_front')
    left_front_jsondir = osp.join(resroot, 'image_record', 'image_record_json', 'left_front')
    left_rear_imgdir = osp.join(resroot, 'left_rear')
    left_rear_jsondir = osp.join(resroot, 'image_record', 'image_record_json', 'left_rear')
    right_front_imgdir = osp.join(resroot, 'right_front')
    right_front_jsondir = osp.join(resroot, 'image_record', 'image_record_json', 'right_front')
    right_rear_imgdir = osp.join(resroot, 'right_rear')
    right_rear_jsondir = osp.join(resroot, 'image_record', 'image_record_json', 'right_rear')
    fusion_jsondir = osp.join(resroot, 'image_record', 'image_record_json', 'fusion')
    
    namelist = list(os.listdir(left_front_imgdir))
    namelist.sort(key=sort_rule)

    crop_config = (0, 200, 120, 0) # desay_corp_top, desay_crop_bottom, nm_crop_top, nm_crop_bottom
    bev_range_config=(100, 40) # h, w
    ego_car_size=(20, 40) # w, h
    scale=0.1 #meter to pixel, 1m = 10pix

    cv2.namedWindow('Replay')
    pause = True
    trackers = {}
    while(1):
        if save_flag:
            savedir = gendir(dst_dir)
        imgname = namelist[file_index]
        imgid = int(imgname.split('.')[0])
        left_front_img, left_front_bbox_infos = process_image_info(left_front_imgdir, left_front_jsondir, imgname, crop_config)
        left_rear_img, left_rear_bbox_infos = process_image_info(left_rear_imgdir, left_rear_jsondir, imgname, crop_config)
        right_front_img, right_front_bbox_infos = process_image_info(right_front_imgdir, right_front_jsondir, imgname, crop_config)
        right_rear_img, right_rear_bbox_infos = process_image_info(right_rear_imgdir, right_rear_jsondir, imgname, crop_config)
        fusion_bbox_infos = process_bev_info(fusion_jsondir, imgname)

        ####keep trackers map
        trackers = update_trackers(trackers, fusion_bbox_infos, bev_range_config, scale)
        publish_bbox_infos, trackers = filter_publish_bbox_infos(trackers, bev_range_config)

        if len(dst_tracking_id_list):
            left_front_img = draw_one_track_bbox(left_front_img, left_front_bbox_infos, 'left_front', imgid, dst_tracking_id_list)
            left_rear_img = draw_one_track_bbox(left_rear_img, left_rear_bbox_infos, 'left_rear', imgid, dst_tracking_id_list)
            right_front_img = draw_one_track_bbox(right_front_img, right_front_bbox_infos, 'right_front', imgid, dst_tracking_id_list, True)
            right_rear_img = draw_one_track_bbox(right_rear_img, right_rear_bbox_infos, 'right_rear', imgid, dst_tracking_id_list)
            bev_img = draw_one_track_bev(fusion_bbox_infos, dst_tracking_id_list, bev_range_config, ego_car_size, scale)
            filter_bev_img = draw_one_track_bev(publish_bbox_infos, dst_tracking_id_list, bev_range_config, ego_car_size, scale)
        else: #all
            left_front_img = draw_all_bbox_per_img(left_front_img, left_front_bbox_infos, 'left_front', imgid)
            left_rear_img = draw_all_bbox_per_img(left_rear_img, left_rear_bbox_infos, 'left_rear', imgid)
            right_front_img = draw_all_bbox_per_img(right_front_img, right_front_bbox_infos, 'right_front', imgid, True)
            lright_rear_img = draw_all_bbox_per_img(right_rear_img, right_rear_bbox_infos, 'right_rear', imgid)
            bev_img = draw_all_bev(fusion_bbox_infos, bev_range_config, ego_car_size, scale)
            filter_bev_img = draw_all_bev(publish_bbox_infos, bev_range_config, ego_car_size, scale)

        left_img_draw = cv2.vconcat([left_front_img, left_rear_img])
        right_img_draw = cv2.vconcat([right_front_img, right_rear_img])
        # draw_img = cv2.hconcat([left_img_draw, right_img_draw, bev_img])
        draw_img = cv2.hconcat([left_img_draw, right_img_draw, filter_bev_img, bev_img])

        if save_flag:
            save_path = osp.join(savedir, imgname)
            cv2.imwrite(save_path, draw_img)
        cv2.imshow('Replay', draw_img)
        print("=>  %d / %d" % (file_index+1, len(namelist)))
        if pause:
            k = cv2.waitKey(0)
            if k == ord('z'):
                if file_index == 0:
                    continue
                else:
                    file_index -= 1
            elif k == ord('r'):
                pause = False
            elif k == ord('x'):
                file_index += 1
                if file_index == len(namelist):
                    print('Replay Finish!')
                    break
            elif k == ord('q'):
                print('Quit replay!')
                break
        else:
            k = cv2.waitKey(20)
            file_index += 1
            if file_index == len(namelist):
                print('Replay Finish!')
                break
            if k == ord('p'):
                pause = True
            elif k == ord('q'):
                print('Quit replay!')
                break
    cv2.destroyAllWindows()