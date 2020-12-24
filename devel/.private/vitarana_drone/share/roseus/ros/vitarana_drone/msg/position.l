;; Auto-generated. Do not edit!


(when (boundp 'vitarana_drone::position)
  (if (not (find-package "VITARANA_DRONE"))
    (make-package "VITARANA_DRONE"))
  (shadow 'position (find-package "VITARANA_DRONE")))
(unless (find-package "VITARANA_DRONE::POSITION")
  (make-package "VITARANA_DRONE::POSITION"))

(in-package "ROS")
;;//! \htmlinclude position.msg.html


(defclass vitarana_drone::position
  :super ros::object
  :slots (_lat _long _alt ))

(defmethod vitarana_drone::position
  (:init
   (&key
    ((:lat __lat) 0.0)
    ((:long __long) 0.0)
    ((:alt __alt) 0.0)
    )
   (send-super :init)
   (setq _lat (float __lat))
   (setq _long (float __long))
   (setq _alt (float __alt))
   self)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:long
   (&optional __long)
   (if __long (setq _long __long)) _long)
  (:alt
   (&optional __alt)
   (if __alt (setq _alt __alt)) _alt)
  (:serialization-length
   ()
   (+
    ;; float64 _lat
    8
    ;; float64 _long
    8
    ;; float64 _alt
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _long
       (sys::poke _long (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alt
       (sys::poke _alt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _long
     (setq _long (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alt
     (setq _alt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get vitarana_drone::position :md5sum-) "24a51b5bef95cbf4999116f3fd0f827c")
(setf (get vitarana_drone::position :datatype-) "vitarana_drone/position")
(setf (get vitarana_drone::position :definition-)
      "float64 lat
float64 long
float64 alt

")



(provide :vitarana_drone/position "24a51b5bef95cbf4999116f3fd0f827c")


