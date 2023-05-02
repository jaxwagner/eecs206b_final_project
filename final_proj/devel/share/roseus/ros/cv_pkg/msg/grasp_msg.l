;; Auto-generated. Do not edit!


(when (boundp 'cv_pkg::grasp_msg)
  (if (not (find-package "CV_PKG"))
    (make-package "CV_PKG"))
  (shadow 'grasp_msg (find-package "CV_PKG")))
(unless (find-package "CV_PKG::GRASP_MSG")
  (make-package "CV_PKG::GRASP_MSG"))

(in-package "ROS")
;;//! \htmlinclude grasp_msg.msg.html


(defclass cv_pkg::grasp_msg
  :super ros::object
  :slots (_grasp_x_cm _grasp_y_cm _grasp_theta ))

(defmethod cv_pkg::grasp_msg
  (:init
   (&key
    ((:grasp_x_cm __grasp_x_cm) 0.0)
    ((:grasp_y_cm __grasp_y_cm) 0.0)
    ((:grasp_theta __grasp_theta) 0.0)
    )
   (send-super :init)
   (setq _grasp_x_cm (float __grasp_x_cm))
   (setq _grasp_y_cm (float __grasp_y_cm))
   (setq _grasp_theta (float __grasp_theta))
   self)
  (:grasp_x_cm
   (&optional __grasp_x_cm)
   (if __grasp_x_cm (setq _grasp_x_cm __grasp_x_cm)) _grasp_x_cm)
  (:grasp_y_cm
   (&optional __grasp_y_cm)
   (if __grasp_y_cm (setq _grasp_y_cm __grasp_y_cm)) _grasp_y_cm)
  (:grasp_theta
   (&optional __grasp_theta)
   (if __grasp_theta (setq _grasp_theta __grasp_theta)) _grasp_theta)
  (:serialization-length
   ()
   (+
    ;; float32 _grasp_x_cm
    4
    ;; float32 _grasp_y_cm
    4
    ;; float32 _grasp_theta
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _grasp_x_cm
       (sys::poke _grasp_x_cm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grasp_y_cm
       (sys::poke _grasp_y_cm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grasp_theta
       (sys::poke _grasp_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _grasp_x_cm
     (setq _grasp_x_cm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grasp_y_cm
     (setq _grasp_y_cm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grasp_theta
     (setq _grasp_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cv_pkg::grasp_msg :md5sum-) "4d3c4f7d62e5e5364236dabf83a36004")
(setf (get cv_pkg::grasp_msg :datatype-) "cv_pkg/grasp_msg")
(setf (get cv_pkg::grasp_msg :definition-)
      "float32 grasp_x_cm
float32 grasp_y_cm
float32 grasp_theta



")



(provide :cv_pkg/grasp_msg "4d3c4f7d62e5e5364236dabf83a36004")


