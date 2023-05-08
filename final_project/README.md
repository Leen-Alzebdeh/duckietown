## AUTONOMOUS DRIVING AND PARKING IN DUCKIETOWN

Authors: Leen Alzebdeh, Tural Bakhtiyarli and Tianming Han

The ObjectDetection repo can be built with "dts devel build -f -H csc229XX" and run with "dts devel run -R csc229XX". The Pj412 repo can be built with "dts devel build -f -H csc229XX" and run with "dts devel run -H csc229XX".

Link to lab report: [https://leen-alzebdeh.github.io/projects/412_final_project/](https://leen-alzebdeh.github.io/projects/412_final_project/)

## Objective

In this project, we have three separate tasks for the robot to complete in one go. To solve these tasks we would like to implement Apriltag detection, obstacle avoidance and object detection. The robot will follow the lane and handle intersections in stage 1, avoid a broken robot in the middle of the road in stage 2 and park inside one of the four parking slots specified in advance in stage 3.

The object detection node should be run on a separate laptop using -R option before running Pj412 repo, and the object detection node uses the following path to access the final model: rospack.get_path("detectron2_duckiebot_node") + "/src/model_final.pth" so make sure to put the model_final.pth file inside the src folder before running "dts devel build -f". The object detection node will publish its detection results onto an image topic for debugging, as well as in json format to another topic containing information about the bounding boxes.

The Pj412 repo can be built with "dts devel build -f -H csc229XX" and run with "dts devel run -H csc229XX"
