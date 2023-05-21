;; Auto-generated. Do not edit!


(when (boundp 'parc_robot::RobotStatus)
  (if (not (find-package "PARC_ROBOT"))
    (make-package "PARC_ROBOT"))
  (shadow 'RobotStatus (find-package "PARC_ROBOT")))
(unless (find-package "PARC_ROBOT::ROBOTSTATUS")
  (make-package "PARC_ROBOT::ROBOTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RobotStatus.msg.html


(defclass parc_robot::RobotStatus
  :super ros::object
  :slots (_status ))

(defmethod parc_robot::RobotStatus
  (:init
   (&key
    ((:status __status) "")
    )
   (send-super :init)
   (setq _status (string __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get parc_robot::RobotStatus :md5sum-) "4fe5af303955c287688e7347e9b00278")
(setf (get parc_robot::RobotStatus :datatype-) "parc_robot/RobotStatus")
(setf (get parc_robot::RobotStatus :definition-)
      "# The status of the robot => \"started\" or \"finished\"
string status

")



(provide :parc_robot/RobotStatus "4fe5af303955c287688e7347e9b00278")


