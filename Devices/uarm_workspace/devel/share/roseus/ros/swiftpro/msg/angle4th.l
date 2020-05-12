;; Auto-generated. Do not edit!


(when (boundp 'swiftpro::angle4th)
  (if (not (find-package "SWIFTPRO"))
    (make-package "SWIFTPRO"))
  (shadow 'angle4th (find-package "SWIFTPRO")))
(unless (find-package "SWIFTPRO::ANGLE4TH")
  (make-package "SWIFTPRO::ANGLE4TH"))

(in-package "ROS")
;;//! \htmlinclude angle4th.msg.html


(defclass swiftpro::angle4th
  :super ros::object
  :slots (_angle4th ))

(defmethod swiftpro::angle4th
  (:init
   (&key
    ((:angle4th __angle4th) 0.0)
    )
   (send-super :init)
   (setq _angle4th (float __angle4th))
   self)
  (:angle4th
   (&optional __angle4th)
   (if __angle4th (setq _angle4th __angle4th)) _angle4th)
  (:serialization-length
   ()
   (+
    ;; float64 _angle4th
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle4th
       (sys::poke _angle4th (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle4th
     (setq _angle4th (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get swiftpro::angle4th :md5sum-) "8eecd591854543ff5e9cf583de2d05e6")
(setf (get swiftpro::angle4th :datatype-) "swiftpro/angle4th")
(setf (get swiftpro::angle4th :definition-)
      "float64 angle4th

")



(provide :swiftpro/angle4th "8eecd591854543ff5e9cf583de2d05e6")


