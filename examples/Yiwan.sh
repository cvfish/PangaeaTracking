#!/bin/bash

# loading results
# # ../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking_qi_template.yml
# #../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading_new_normals.yml
# #../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading.yml

# loading results into a buffer first
#../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading_fast.yml

# do real tracking
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking.yml
#../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking_test.yml
#../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking.yml
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking_depthMapInput.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking_depthMapInput.yml
#../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Tshirt_tracking_depthMapInput.yml

# do rigid tracking
# ../build/PangaeaTracking/bin/PangaeaTracking_console_bk ../config/Yiwan_rigid_tracking.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_rigid_tracking.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_rigid_tracking_debug.yml

# load rigid tracking results
../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading_rigid.yml
