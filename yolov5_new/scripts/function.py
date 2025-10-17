from yolov5.msg import YoloRes
import numpy as np
import copy

# function__msg2list: msg_list to list_list
def msg2list(list_msg):
    logo_x_list = []
    logo_y_list = []
    logo_z_list = []
    QR_x_list = []
    QR_y_list = []
    QR_z_list = []
    for list_item in list_msg:
        logo_x_list.append(list_item.logo_pos.x)
        logo_y_list.append(list_item.logo_pos.y)
        logo_z_list.append(list_item.logo_pos.z)
        QR_x_list.append(list_item.QR_pos.x)
        QR_y_list.append(list_item.QR_pos.y)
        QR_z_list.append(list_item.QR_pos.z)
    logo_list = [logo_x_list, logo_y_list, logo_z_list]
    QR_list = [QR_x_list, QR_y_list, QR_z_list]
    return logo_list, QR_list

# function__list2msg: list_list to msg_list
def list2msg(logo_list, QR_list):
    list_msg = []
    for i in range(len(logo_list[0])):
        vision_info = YoloRes()
        vision_info.QR_pos.x = QR_list[0][i]
        vision_info.QR_pos.y = QR_list[1][i]
        vision_info.QR_pos.z = QR_list[2][i]
        vision_info.logo_pos.x = logo_list[0][i]
        vision_info.logo_pos.y = logo_list[1][i]
        vision_info.logo_pos.z = logo_list[2][i]
        list_msg.append(vision_info)
    return list_msg


# function__outlier_detection£º outlier detection
# input 
#   list:       window_list 
#   data_new:   new data
#   n:          sigma filtering weights
# output:
#   window_list new window_list
def outlier_detection(window_list, data_new, n):

    # prior
    QR_prior = True
    logo_prior = True
    for list_item in window_list:
        if(list_item.logo_update == False):
            logo_prior = False
        if(list_item.qr_update == False):
            QR_prior = False

    # define long window
    window_list.append(data_new)
    logo_list = []
    QR_list = []

    # YoloRes.msg_list to list_list
    logo_list, QR_list = msg2list(window_list)

    # calculate std and mean
    logo_std_new = np.std(logo_list, axis=1)
    logo_mean = np.mean(logo_list, axis=1)
    QR_std_new = np.std(QR_list, axis=1)
    QR_mean = np.mean(QR_list, axis=1)

    # calculate length and columns
    length_new = len(logo_list)
    columns_new = len(logo_list[0])

    # evaluate QR/logo valid or not
    QR_valid = True
    logo_valid = True
    for i in range(length_new):
        if abs(logo_list[i][columns_new - 1] - logo_mean[i]) > (n * logo_std_new[i]):
            logo_valid = False
        if abs(QR_list[i][columns_new - 1] - QR_mean[i]) > (n * QR_std_new[i]):
            QR_valid = False

    # replace new data
    window_list_new = []
    if (logo_valid == True and QR_valid == True):
        window_list_new = list2msg(logo_list, QR_list)
    elif(logo_valid == False and QR_valid == False and logo_prior == True and QR_prior == True):
        for i in range(len(logo_list)):
            logo_list[i][len(logo_list[0]) - 1] = logo_list[i][len(logo_list[0]) - 2]        
            QR_list[i][len(QR_list[0]) - 1] = QR_list[i][len(QR_list[0]) - 2] 
        window_list_new = list2msg(logo_list, QR_list)
        print("warn, logo and QR invalid!")
    elif(logo_valid == True and QR_valid == False and QR_prior == True):
        for i in range(len(logo_list)):
            QR_list[i][len(QR_list[0]) - 1] = QR_list[i][len(QR_list[0]) - 2] 
        window_list_new = list2msg(logo_list, QR_list)
        print("warn, QR invalid!")
    elif(logo_valid == False and QR_valid == True and logo_prior == True):
        for i in range(len(logo_list)):
            logo_list[i][len(logo_list[0]) - 1] = logo_list[i][len(logo_list[0]) - 2]        
        window_list_new = list2msg(logo_list, QR_list)
        print("warn, logo invalid!")
    else:
        window_list_new = list2msg(logo_list, QR_list)

    # update flag
    for i in range(len(window_list_new)):
        window_list_new[i].logo_update = window_list[i].logo_update
        window_list_new[i].qr_update = window_list[i].qr_update

    # delete old data
    window_list_new.pop(0)

    # return
    return window_list_new


# function__mean_filter: calculate mean of YoloRes list
# input 
#   window_list: data window list 
#   data_new:    new data
# output:
#   vision_info_filtering:  data with filter
def mean_filter(window_list):

    # initialize logo_list and QR_list
    logo_list = []
    QR_list = []
    window_list_copy = copy.deepcopy(window_list)
    logo_list, QR_list = msg2list(window_list_copy)

    # calculate mean
    logo_mean = np.mean(logo_list, axis=1)
    QR_mean = np.mean(QR_list, axis=1)

    # define new vision info
    vision_info_filtering = YoloRes()
    vision_info_filtering.logo_pos.x = logo_mean[0]
    vision_info_filtering.logo_pos.y = logo_mean[1]
    vision_info_filtering.logo_pos.z = logo_mean[2]
    vision_info_filtering.QR_pos.x = QR_mean[0]
    vision_info_filtering.QR_pos.y = QR_mean[1]
    vision_info_filtering.QR_pos.z = QR_mean[2]
    vision_info_filtering.logo_update = window_list[len(window_list) - 1].logo_update
    vision_info_filtering.qr_update = window_list[len(window_list) - 1].qr_update

    # return
    return vision_info_filtering







