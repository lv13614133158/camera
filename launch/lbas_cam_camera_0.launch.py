<launch>
    <rosparam file="$(find lbas_cam)/config/camera_0.yaml" command="load" ns="camera_0"/>
    
    <node pkg="my_libas_cam" exec="my_libas_cam" name="my_libas_cam" output="screen">
    <param name="param_name" value="param_value" />
    </node>
</launch>