#!/bin/bash

# loading results
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading.yml
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading_new_normals.yml
#../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading.yml

# loading results into a buffer first
#../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_loading_fast.yml

# do real tracking
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking.yml
#../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking_test.yml
#../build_debug/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking.yml
#../build/PangaeaTracking/bin/PangaeaTracking ../config/Yiwan_tracking_depthMapInput.yml
# ../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Yiwan_tracking_depthMapInput.yml
../build/PangaeaTracking/bin/PangaeaTracking_console ../config/Tshirt_tracking_depthMapInput.yml
