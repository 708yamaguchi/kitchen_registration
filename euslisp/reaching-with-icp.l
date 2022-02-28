;; Usage

;; 1. PR2 reaches hand to *target-coords* and reset pose.
;;   $ (reaching-before-icp)
;; 2. Save pcd and rename.
;;   $ roslaunch kitchen_registration pointcloud_to_pcd.launch
;;   $ roscd kitchen_registration/data; mv xxx.pcd base_link.pcd
;; 3. Move PR2 base by joystick teleop
;; 4. PR2 reaches hand to *target-coords* with icp.
;;   $ roslaunch kitchen_registration icp_registration.launch
;;   $ (reaching-after-icp)

;; If ICP is good enough, PR2 hand in 1. and 4. should reaches the same coords.
;; Note that PCD save and ICP registration should be done when PR2 is :reset-pose

(load "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)

(setq *target-coords* (make-coords :pos #f(700 -300 800)))

(defun reaching-before-icp ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :reset-pose)
  (send *pr2* :rarm :inverse-kinematics
	*target-coords*
	:use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (send *ri* :wait-interpolation))

(defun reaching-after-icp ()
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (send *ri* :wait-interpolation)
  (format t "Waiting for ICP registration result...~%")
  (let ((icp-score 10000) icp-result after-move-coords)
    (while (> icp-score 0.3)
      (format t "Got ICP registration result, but accuracy is low. Retry.... ~%")
      (setq icp-result (one-shot-subscribe "/icp_registration/icp_result"
					   jsk_recognition_msgs::ICPResult))
      (setq icp-score (send icp-result :score)))
    (setq after-move-coords (ros::tf-pose->coords (send icp-result :pose)))
    (setq after-move-coords (send after-move-coords :inverse-transformation))
    (format t "PR2 moves by ~A~%" after-move-coords)
    (send *pr2* :move-to after-move-coords :world))
  (send *pr2* :reset-pose)
  (send *pr2* :rarm :inverse-kinematics
	*target-coords*
	:use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation))
