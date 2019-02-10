-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here
 
    pos_hoop3_handle = sim.getObjectHandle('Position_hoop3')

    orient_hoop3_handle=sim.getObjectHandle('Orientation_hoop3')
    drone_handle=sim.getObjectHandle('Drone_Pos_Emulation')
    start_handle=sim.getObjectHandle('Dummy')

    flag=0
    cnt=0
    a = {x = 0, y = 0, w = 0, z = 0}
    b = {x = 0, y = 0, z = 0}
    pose = {position = b, orientation = a, }
    -- Subscribing to the required topics 
    --aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

end


function sysCall_actuation()
      if flag == 1 then
         if cnt ==1 then 
            result2 = sim.setObjectQuaternion(orient_hoop3_handle,-1,{-pose.orientation.y,-pose.orientation.x,pose.orientation.w,pose.orientation.z})
            result1 = sim.setObjectPosition(pos_hoop3_handle,-1,{-pose.position.y*0.106 ,-pose.position.x*0.171,2.78-pose.position.z*0.097})
         end
         if cnt==2 then
            result2 = sim.setObjectPosition(drone_handle,-1,{-pose.position.y*0.106 ,-pose.position.x*0.171,2.78-pose.position.z*0.097})
            result1 = sim.setObjectPosition(start_handle,-1,{-pose.position.y*0.106 ,-pose.position.x*0.171,2.78-pose.position.z*0.097})
            
         end
        flag=0
      end
end
   

function sysCall_sensing()
    -- put your sensing code here
      if cnt>=2 and flag==0 then
          result1 = sim.setObjectPosition(drone_handle,-1,{-pose.position.y*0.106 ,-pose.position.x*0.171,2.78-pose.position.z*0.097})
      end
end

function sysCall_cleanup()
    -- do some clean-up here
            
end

function aruco_callback(msg)
    pose.orientation=msg.markers[1].pose.pose.orientation
end

function whycon_callback(msg)
    pose.position=msg.poses[1].position    
end

function key_callback(msg)
    if msg.data == 500 then
        flag=1
        cnt=cnt+1
    end
end-- This script is used for realtime emulation of the environment in V-REP

