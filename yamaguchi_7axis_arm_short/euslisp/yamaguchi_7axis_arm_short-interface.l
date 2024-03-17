(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "kxr_controller")

;;;
;;; kxr robot interface
;;;


(defclass kxr-interface
  :super robot-interface
  :slots (joint-names servo-on-off-client))

(defmethod kxr-interface
  (:init (robot &rest args &key (type :default-controller) &allow-other-keys)
   (let* ((namespace (cadr (memq :namespace args)))
          (joint-param (if namespace (format nil "~A/~A" namespace "/kxr_fullbody_controller/joints")
                           "/kxr_fullbody_controller/joints")))
     (setq joint-names (ros::get-param joint-param nil))
     (while (and (ros::ok) (null joint-names))
       (ros::ros-info (format nil "Waiting ~A rosparam set" joint-param))
       (setq joint-names (ros::get-param joint-param nil))
       (ros::sleep)))

   (send-super* :init :robot robot :type type
                :groupname "kxr_interface"
                args)

   (setq servo-on-off-client (instance ros::simple-action-client :init
                                       (if namespace (format nil "~A/~A" namespace "/kxr_fullbody_controller/servo_on_off") "/kxr_fullbody_controller/servo_on_off")
                                       kxr_controller::ServoOnOffAction
                                       :groupname groupname))
   (dolist (action (list servo-on-off-client))
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, kxr-interface is disabled" action)
       (return)))
   t)
  (:default-controller
   ()
   (send self :fullbody-controller))
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "kxr_fullbody_controller/follow_joint_trajectory")
     (cons :controller-state "kxr_fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names joint-names))))
  (:servo-on
   (&key (names nil))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-on t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     ;; send current angle-vector
     (send self :angle-vector (send self :state :potentio-vector) 0.1)
     (send self :wait-interpolation)

     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element t))
     (send servo-on-off-client :send-goal goal)))
  (:servo-off
   (&key (names nil))
   (unless joint-action-enable
     (if viewer (send self :draw-objects))
     (return-from :servo-off t))
   (when (null names)
     (setq names joint-names))
   (let* (goal result)
     (setq goal (instance kxr_controller::ServoOnOffGoal :init))
     (send goal :joint_names names)
     (send goal :servo_on_states (make-array (length names) :initial-element nil))
     (send servo-on-off-client :send-goal goal))))

(load "package://yamaguchi_7axis_arm_short/euslisp/yamaguchi_7axis_arm_short.l")

(ros::roseus "kxr_interface")
(setq *robot* (yamaguchi_7axis_arm_short))
;; (setq *ri* (instance kxr-interface :init *robot* :namespace "robot_a"))
(setq *ri* (instance kxr-interface :init *robot*))
(ros::spin-once)
(send *ri* :spin-once)
;; (send *ri* :angle-vector (send *robot* :angle-vector))
