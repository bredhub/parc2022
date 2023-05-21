;; Auto-generated. Do not edit!


(when (boundp 'parc_robot::WeedDetection)
  (if (not (find-package "PARC_ROBOT"))
    (make-package "PARC_ROBOT"))
  (shadow 'WeedDetection (find-package "PARC_ROBOT")))
(unless (find-package "PARC_ROBOT::WEEDDETECTION")
  (make-package "PARC_ROBOT::WEEDDETECTION"))

(in-package "ROS")
;;//! \htmlinclude WeedDetection.msg.html


(defclass parc_robot::WeedDetection
  :super ros::object
  :slots (_weeds ))

(defmethod parc_robot::WeedDetection
  (:init
   (&key
    ((:weeds __weeds) ())
    )
   (send-super :init)
   (setq _weeds __weeds)
   self)
  (:weeds
   (&rest __weeds)
   (if (keywordp (car __weeds))
       (send* _weeds __weeds)
     (progn
       (if __weeds (setq _weeds (car __weeds)))
       _weeds)))
  (:serialization-length
   ()
   (+
    ;; parc_robot/Weed[] _weeds
    (apply #'+ (send-all _weeds :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; parc_robot/Weed[] _weeds
     (write-long (length _weeds) s)
     (dolist (elem _weeds)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; parc_robot/Weed[] _weeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _weeds (let (r) (dotimes (i n) (push (instance parc_robot::Weed :init) r)) r))
     (dolist (elem- _weeds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get parc_robot::WeedDetection :md5sum-) "845a4e3a48d0573bc750910e15745135")
(setf (get parc_robot::WeedDetection :datatype-) "parc_robot/WeedDetection")
(setf (get parc_robot::WeedDetection :definition-)
      "# A list of detected weeds
Weed[] weeds


================================================================================
MSG: parc_robot/Weed
# A weed in the field
float64 latitude
float64 longitude

")



(provide :parc_robot/WeedDetection "845a4e3a48d0573bc750910e15745135")


