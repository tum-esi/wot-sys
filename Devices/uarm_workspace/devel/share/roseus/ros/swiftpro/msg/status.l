;; Auto-generated. Do not edit!


(when (boundp 'swiftpro::status)
  (if (not (find-package "SWIFTPRO"))
    (make-package "SWIFTPRO"))
  (shadow 'status (find-package "SWIFTPRO")))
(unless (find-package "SWIFTPRO::STATUS")
  (make-package "SWIFTPRO::STATUS"))

(in-package "ROS")
;;//! \htmlinclude status.msg.html


(defclass swiftpro::status
  :super ros::object
  :slots (_status ))

(defmethod swiftpro::status
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get swiftpro::status :md5sum-) "284aa12dd9e9e760802ac9f38036ea5e")
(setf (get swiftpro::status :datatype-) "swiftpro/status")
(setf (get swiftpro::status :definition-)
      "uint8 status

")



(provide :swiftpro/status "284aa12dd9e9e760802ac9f38036ea5e")


