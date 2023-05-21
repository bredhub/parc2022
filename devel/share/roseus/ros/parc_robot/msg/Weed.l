;; Auto-generated. Do not edit!


(when (boundp 'parc_robot::Weed)
  (if (not (find-package "PARC_ROBOT"))
    (make-package "PARC_ROBOT"))
  (shadow 'Weed (find-package "PARC_ROBOT")))
(unless (find-package "PARC_ROBOT::WEED")
  (make-package "PARC_ROBOT::WEED"))

(in-package "ROS")
;;//! \htmlinclude Weed.msg.html


(defclass parc_robot::Weed
  :super ros::object
  :slots (_latitude _longitude ))

(defmethod parc_robot::Weed
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get parc_robot::Weed :md5sum-) "680c6dc7da65a2421a822205dcbdb600")
(setf (get parc_robot::Weed :datatype-) "parc_robot/Weed")
(setf (get parc_robot::Weed :definition-)
      "# A weed in the field
float64 latitude
float64 longitude

")



(provide :parc_robot/Weed "680c6dc7da65a2421a822205dcbdb600")


